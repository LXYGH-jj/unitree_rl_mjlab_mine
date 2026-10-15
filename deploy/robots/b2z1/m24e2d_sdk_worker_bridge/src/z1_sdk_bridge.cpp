// M2.4e2h: Z1 SDK bridge for standalone worker; PHYSICAL I/O requires supervised bench.
// Hardware start(), exchange() both affect motors. NEVER treat as read-only.
// No recv timestamp provenance; feedback is explicitly getter sampled.
#include "z1_sdk_bridge.h"
#include "m24e2h_gravity_guard.h"
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/arm_common.h>
#include <stdexcept>
#include <cmath>
#include <array>
#include <iostream>
#include <string>
#include <thread>
namespace m24e2b {
namespace {
// M2.4e2k conservative DEVELOPMENT-only command envelope for a supported,
// unloaded, low-speed Z1 V2 bench. These numbers are NOT Unitree per-joint
// continuous torque ratings and cannot bound actuator current in hardware.
constexpr double kBenchEstimatedTotalNm = 20.0;
constexpr double kBenchObservedTauNm = 20.0;
constexpr double kBenchTargetErrorRad = 0.035;
constexpr double kBenchDesiredVelocityRadS = 0.10;
constexpr double kBenchMeasuredVelocityRadS = 0.30;
constexpr int kBenchTemperatureC = 60;

void checkBenchCommand(const UNITREE_ARM::unitreeArm& arm,
                       const ::Vec6& q, const ::Vec6& dq, const ::Vec6& tau_ff,
                       const char* phase) {
    const auto* comp=arm._ctrlComp;
    const auto* feedback=arm.lowstate;
    const auto* cmd=arm.lowcmd;
    if(!comp || !comp->udp || comp->udp->isDisConnect || !feedback || !cmd ||
       feedback->q.size()<6 || feedback->dq.size()<6 || feedback->tau.size()<6 ||
       feedback->temperature.size()<6 || feedback->errorstate.size()<6 ||
       feedback->isMotorConnected.size()<6 || cmd->kp.size()<6 || cmd->kd.size()<6)
        throw std::runtime_error("Z1 bench torque guard: feedback/gains unavailable");
    for(int i=0;i<6;++i) {
        const double measured_q=feedback->q[i];
        const double measured_dq=feedback->dq[i];
        const double measured_tau=feedback->tau[i];
        const double kp=cmd->kp[i],kd=cmd->kd[i];
        if(!std::isfinite(q[i]) || !std::isfinite(dq[i]) ||
           !std::isfinite(tau_ff[i]) || !std::isfinite(measured_q) ||
           !std::isfinite(measured_dq) || !std::isfinite(measured_tau) ||
           !std::isfinite(kp) || !std::isfinite(kd) || kp<=0 || kd<=0)
            throw std::runtime_error("Z1 bench guard: nonfinite joint data/gains J"+std::to_string(i+1));
        if(feedback->isMotorConnected[i]!=0 ||
           (feedback->errorstate[i] & static_cast<uint8_t>(~0x40u)) != 0 ||
           feedback->temperature[i]>=kBenchTemperatureC)
            throw std::runtime_error("Z1 bench guard: motor fault/temperature J"+std::to_string(i+1));
        if(std::abs(measured_tau)>kBenchObservedTauNm)
            throw std::runtime_error("Z1 bench guard: observed torque J"+std::to_string(i+1)+
                                     " measured="+std::to_string(measured_tau)+
                                     " limit="+std::to_string(kBenchObservedTauNm));
        const double position_error=q[i]-measured_q;
        const bool position_exceeded=std::abs(position_error)>kBenchTargetErrorRad;
        const bool command_velocity_exceeded=std::abs(dq[i])>kBenchDesiredVelocityRadS;
        const bool measured_velocity_exceeded=std::abs(measured_dq)>kBenchMeasuredVelocityRadS;
        if(position_exceeded || command_velocity_exceeded || measured_velocity_exceeded) {
            throw std::runtime_error(
                "Z1 bench guard: position/velocity envelope J"+std::to_string(i+1)+
                " phase="+phase+
                " position_exceeded="+std::to_string(position_exceeded)+
                " command_velocity_exceeded="+std::to_string(command_velocity_exceeded)+
                " measured_velocity_exceeded="+std::to_string(measured_velocity_exceeded)+
                " q_cmd="+std::to_string(q[i])+
                " q_meas="+std::to_string(measured_q)+
                " q_error="+std::to_string(position_error)+
                " q_error_limit="+std::to_string(kBenchTargetErrorRad)+
                " dq_cmd="+std::to_string(dq[i])+
                " dq_cmd_limit="+std::to_string(kBenchDesiredVelocityRadS)+
                " dq_meas="+std::to_string(measured_dq)+
                " dq_meas_limit="+std::to_string(kBenchMeasuredVelocityRadS)+
                " tau_meas="+std::to_string(measured_tau)+
                " tau_ff="+std::to_string(tau_ff[i]));
        }
        // Z1 V2 protocol scales kp, kd before applying PD.
        // Prediction uses the most recent SDK getter sample, NOT proven fresh RX,
        // so this guard is never a hard actuator torque/current limit.
        const double predicted = 25.6*kp*(q[i]-measured_q) +
                                 0.0128*kd*(dq[i]-measured_dq) + tau_ff[i];
        if(!std::isfinite(predicted) || std::abs(predicted)>kBenchEstimatedTotalNm) {
            const double tau_pos=25.6*kp*(q[i]-measured_q);
            const double tau_vel=0.0128*kd*(dq[i]-measured_dq);
            throw std::runtime_error(
                "Z1 bench guard: estimated total torque limit at J"+std::to_string(i+1)+
                " predicted="+std::to_string(predicted)+
                " limit="+std::to_string(kBenchEstimatedTotalNm)+
                " position_pd="+std::to_string(tau_pos)+
                " velocity_pd="+std::to_string(tau_vel)+
                " gravity_ff="+std::to_string(tau_ff[i])+
                " feedback_tau="+std::to_string(measured_tau)+
                " q_cmd="+std::to_string(q[i])+
                " q_meas="+std::to_string(measured_q)+
                " dq_cmd="+std::to_string(dq[i])+
                " dq_meas="+std::to_string(measured_dq)+
                " kp_sdk="+std::to_string(kp)+
                " kd_sdk="+std::to_string(kd));
        }
    }
}
} // namespace
class SdkBridge::Impl {
public:
    explicit Impl(bool with_gripper, GainProfile profile):with_gripper_(with_gripper),gain_profile_(profile){}
    bool with_gripper_{false};
    GainProfile gain_profile_{GainProfile::VendorDefault};
    bool started_{false};
    bool stop_attempted_{false};
    bool vendor_thread_running_{false};
    std::unique_ptr<UNITREE_ARM::unitreeArm> arm_;
    std::thread::id owner_{};
    Feedback initial_{};
    // SDK background thread used ONLY during mode transition, never during exchange().
};
SdkBridge::SdkBridge(bool has_gripper, GainProfile profile):impl_(std::make_unique<Impl>(has_gripper,profile)){}
SdkBridge::~SdkBridge(){
    // Covers partial start() failure as well as ordinary lifetime end.
    // This is a best-effort vendor FSM handoff, NOT a physical safety guarantee.
    if(impl_ && impl_->arm_) (void)stop();
}
bool SdkBridge::started() const noexcept {return impl_->started_;}
bool SdkBridge::stop() noexcept {
#ifndef M24E2B_ENABLE_HARDWARE_SDK
    return false;
#else
    if(!impl_->arm_ || impl_->stop_attempted_) return false;
    impl_->stop_attempted_=true;
    auto& arm=*impl_->arm_;
    // No attempts to drive a return-to-home trajectory on shutdown.
    // A lost UDP connection cannot be made safe by sending more UDP packets.
    bool passive_confirmed=false;
    try {
        if(arm._ctrlComp && arm._ctrlComp->udp &&
           !arm._ctrlComp->udp->isDisConnect) {
            if(!impl_->vendor_thread_running_ && arm.sendRecvThread) {
                arm.sendRecvThread->start();
                impl_->vendor_thread_running_=true;
            }
            // Vendor setFsm needs the SDK communication loop to remain active.
            // Do not assume this transition succeeds: report the result.
            passive_confirmed=arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE) &&
                arm._ctrlComp->recvState.state==UNITREE_ARM::ArmFSMState::PASSIVE;
        }
    } catch (...) { passive_confirmed=false; }
    if(impl_->vendor_thread_running_ && arm.sendRecvThread) {
        try {arm.sendRecvThread->shutdown();} catch (...) {}
        impl_->vendor_thread_running_=false;
    }
    impl_->started_=false;
    return passive_confirmed;
#endif
}
void SdkBridge::start(){
#ifndef M24E2B_ENABLE_HARDWARE_SDK
    throw std::runtime_error("M2.4e2b real SDK construction LOCKED; compile-check only");
#else
    if(impl_->started_)throw std::runtime_error("SDK already started");
    impl_->owner_=std::this_thread::get_id();
    impl_->arm_=std::make_unique<UNITREE_ARM::unitreeArm>(impl_->with_gripper_);
    auto& arm=*impl_->arm_;
    // Do not use this prototype on hardware without operator-attended bench qualification.
    arm.sendRecvThread->start();
    impl_->vendor_thread_running_=true;
    if(!arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE))
        throw std::runtime_error("cannot enter PASSIVE");
    if(!arm.lowstate || arm.lowstate->q.size()<6 || arm.lowstate->dq.size()<6 ||
       !arm._ctrlComp || !arm._ctrlComp->udp || arm._ctrlComp->udp->isDisConnect)
        throw std::runtime_error("initial SDK feedback unavailable");
    ::Vec6 initial=::Vec6::Zero();
    for(int i=0;i<6;i++){
        if(!std::isfinite(arm.lowstate->q[i]))throw std::runtime_error("invalid initial q");
        initial[i]=arm.lowstate->q[i];
    }
    if(!arm._ctrlComp->armModel) throw std::runtime_error("Z1 armModel unavailable before LOWCMD");
    // PASSIVE gains may be zero. That is NOT a LOWCMD initialization fault.
    // Initialize with the official C++ SDK defaults before the LOWCMD transition
    // so that the first low-level command has explicitly configured gains.
    // Do not mix in ALORE Python binding's differently-scaled gains.
    if(!arm.lowcmd) throw std::runtime_error("Z1 lowcmd unavailable");
    arm.lowcmd->setControlGain();
    const auto validate_gains=[&arm]() {
        if(arm.lowcmd->kp.size()<6 || arm.lowcmd->kd.size()<6)
            throw std::runtime_error("Z1 LOWCMD PD gains unavailable");
        for(int i=0;i<6;++i) {
            const double kp=arm.lowcmd->kp[i], kd=arm.lowcmd->kd[i];
            if(!std::isfinite(kp) || !std::isfinite(kd) || kp<=0 || kd<=0)
                throw std::runtime_error("Z1 LOWCMD PD gains nonpositive/nonfinite at joint "+std::to_string(i+1));
        }
    };
    validate_gains();
    ::Vec6 model_tau=arm._ctrlComp->armModel->inverseDynamics(
        initial, ::Vec6::Zero(), ::Vec6::Zero(), ::Vec6::Zero());
    m24e2h::Vec6 measured_g{}, user_tau{};
    for(int i=0;i<6;++i)measured_g[i]=model_tau[i];
    const auto guarded=m24e2h::combineGravityTorque(measured_g,user_tau);
    ::Vec6 compensated=::Vec6::Zero();
    for(int i=0;i<6;++i)compensated[i]=guarded[i];
    std::cerr << "[M2.4e2k] SDK model startup feedforward Nm:";
    for(int i=0;i<6;++i)std::cerr << " J" << i+1 << "=" << compensated[i];
    std::cerr << '\n';
    // Observe SDK getter snapshots across the official FSM transition.
    // One-time diagnostics only: NO new SDK sendRecv, gain changes or motor commands.
    const auto log_joint3=[&arm](const char* phase) {
        if(!arm.lowstate || arm.lowstate->q.size()<3 ||
           arm.lowstate->dq.size()<3 || arm.lowstate->tau.size()<3) return;
        std::cerr << "[M2.4e2k] J3 transition " << phase
                  << " q=" << arm.lowstate->q[2]
                  << " dq=" << arm.lowstate->dq[2]
                  << " tau_feedback=" << arm.lowstate->tau[2] << '\n';
    };
    log_joint3("before-lowcmd");
    checkBenchCommand(arm,initial,::Vec6::Zero(),compensated,"startup-passive");
    arm.setArmCmd(initial,::Vec6::Zero(),compensated);
    if(!arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD))
        throw std::runtime_error("LOWCMD transition failed");
    log_joint3("after-setFsm");
    arm.sendRecvThread->shutdown();  // exclusive sendRecv owner after this point
    impl_->vendor_thread_running_=false;
    log_joint3("after-sdk-thread-stop");
    // Observed vendor gains after setFsm(LOWCMD); configure ALORE ONLY after
    // this transition, with the SDK background sender stopped.
    if(impl_->gain_profile_==GainProfile::AloreV2) {
        // ALORE Deployment/Z1_deploy/z1_control.py:
        // desired effective Kp=[264,328,264,264,264,264],
        // effective Kd=[1.5,3.0,1.5,1.5,1.5,1.5].
        // Z1 controller V2 uses Kp*25.6 and Kd*0.0128 on the wire.
        // Preserve any existing gripper gain by only replacing joints 1..6.
        if(arm.lowcmd->kp.size()<6 || arm.lowcmd->kd.size()<6)
            throw std::runtime_error("Z1 LOWCMD gains unavailable for ALORE profile");
        auto kp=arm.lowcmd->kp;
        auto kd=arm.lowcmd->kd;
        const double kp_effective[6]={264.,328.,264.,264.,264.,264.};
        const double kd_effective[6]={1.5,3.0,1.5,1.5,1.5,1.5};
        for(int i=0;i<6;++i) {
            kp[i]=kp_effective[i]/25.6;
            kd[i]=kd_effective[i]/0.0128;
        }
        arm.lowcmd->setControlGain(kp,kd);
        // After setFsm(), the SDK may reset the gains to vendor defaults.
        // Verify the post-transition object values BEFORE any Worker exchange().
        for(int i=0;i<6;++i) {
            if(std::abs(arm.lowcmd->kp[i]-kp[i])>1e-8 ||
               std::abs(arm.lowcmd->kd[i]-kd[i])>1e-8)
                throw std::runtime_error("ALORE V2 LOWCMD gain assignment failed at J"+
                                         std::to_string(i+1));
        }
    }
    validate_gains();
    std::cerr << "[M2.4e2k] SDK LOWCMD transition returned success; gain_profile="
              << (impl_->gain_profile_==GainProfile::AloreV2?"alore-v2":"vendor")
              << " (SDK protocol Kp/Kd):";
    for(int i=0;i<6;++i)
        std::cerr << " J" << i+1 << "=" << arm.lowcmd->kp[i] << "/" << arm.lowcmd->kd[i];
    std::cerr << '\n';
    std::cerr << "[M2.4e2k] DEVELOPMENT bench guard (PROVISIONAL, not motor rating):"
              << " |estimated_total_tau|<=" << kBenchEstimatedTotalNm << "Nm,"
              << " |feedback_tau|<=" << kBenchObservedTauNm << "Nm,"
              << " |qcmd-q|<=" << kBenchTargetErrorRad << "rad,"
              << " |qd_cmd|<=" << kBenchDesiredVelocityRadS << "rad/s,"
              << " |qd_meas|<=" << kBenchMeasuredVelocityRadS << "rad/s,"
              << " temperature<" << kBenchTemperatureC << "C."
              << " NOT hardware current/torque limiting.\n";
    // Capture actual, measured startup joints. Never use ALORE default pose as real state.
    // Timestamp represents SDK getter sampling, not an independently proven UDP RX.
    const auto* l=arm.lowstate;
    if(!l || l->q.size()<6 || l->dq.size()<6 || l->tau.size()<6 ||
       l->temperature.size()<6 || l->errorstate.size()<6 ||
       l->isMotorConnected.size()<6)
        throw std::runtime_error("incomplete Z1 initial feedback");
    Feedback f{};
    f.sdk_connected=!arm._ctrlComp->udp->isDisConnect;
    f.lowcmd_reported=arm._ctrlComp->recvState.state==UNITREE_ARM::ArmFSMState::LOWCMD;
    f.read_at=Clock::now();
    for(int i=0;i<6;i++){
        f.q[i]=l->q[i];f.dq[i]=l->dq[i];f.tau[i]=l->tau[i];
        f.temperature[i]=l->temperature[i]; f.motor_error[i]=static_cast<int>(l->errorstate[i] & static_cast<uint8_t>(~0x40u)); // SDK bit 0x40: Ignore
        f.motor_connection[i]=l->isMotorConnected[i];
        if(!std::isfinite(f.q[i])||!std::isfinite(f.dq[i])||!std::isfinite(f.tau[i]))
            throw std::runtime_error("nonfinite Z1 startup joint feedback");
    }
    if(!f.sdk_connected || !f.lowcmd_reported)
        throw std::runtime_error("Z1 LOWCMD not confirmed after startup");
    impl_->initial_=f;
    impl_->started_=true;
#endif
}
Feedback SdkBridge::initialFeedback() const {
    if(!impl_->started_ || std::this_thread::get_id()!=impl_->owner_)
        throw std::runtime_error("initialFeedback requires started SDK and owning thread");
    return impl_->initial_;
}
Feedback SdkBridge::exchange(const Command& c){
#ifndef M24E2B_ENABLE_HARDWARE_SDK
    (void)c;
    throw std::runtime_error("M2.4e2b physical exchange LOCKED");
#else
    if(!impl_->started_ || std::this_thread::get_id()!=impl_->owner_)
        throw std::runtime_error("SDK sendRecv owner violation");
    for(int i=0;i<6;i++) if(!std::isfinite(c.q[i])||!std::isfinite(c.dq[i])||
       !std::isfinite(c.tau[i]))throw std::invalid_argument("nonfinite SDK command");
    // The hardware log previously showed vendor gains despite alore-v2 being
    // requested. Never continue to send commands with an unexpected profile.
    if(impl_->gain_profile_==GainProfile::AloreV2) {
        const auto* low=impl_->arm_->lowcmd;
        const double expected_kp[6]={264./25.6,328./25.6,264./25.6,
                                     264./25.6,264./25.6,264./25.6};
        const double expected_kd[6]={1.5/0.0128,3.0/0.0128,1.5/0.0128,
                                     1.5/0.0128,1.5/0.0128,1.5/0.0128};
        if(!low || low->kp.size()<6 || low->kd.size()<6)
            throw std::runtime_error("ALORE gain profile lost in SDK LOWCMD");
        for(int i=0;i<6;++i)
            if(!std::isfinite(low->kp[i]) || !std::isfinite(low->kd[i]) ||
               std::abs(low->kp[i]-expected_kp[i])>1e-7 ||
               std::abs(low->kd[i]-expected_kd[i])>1e-7)
                throw std::runtime_error("ALORE gain profile changed while running J"+
                                         std::to_string(i+1));
    }
    ::Vec6 q,qd,tau;
    for(int i=0;i<6;i++){q[i]=c.q[i];qd[i]=c.dq[i];}
    if(!impl_->arm_->_ctrlComp || !impl_->arm_->_ctrlComp->armModel)
        throw std::runtime_error("Z1 armModel unavailable during control");
    const ::Vec6 model_tau=impl_->arm_->_ctrlComp->armModel->inverseDynamics(
        q,qd,::Vec6::Zero(),::Vec6::Zero());
    m24e2h::Vec6 gravity{},extra{};
    for(int i=0;i<6;i++){gravity[i]=model_tau[i];extra[i]=c.tau[i];}
    const auto guarded=m24e2h::combineGravityTorque(gravity,extra);
    for(int i=0;i<6;i++)tau[i]=guarded[i];
    // Before any new low-level command, veto excessive predicted PD+feedforward.
    checkBenchCommand(*impl_->arm_,q,qd,tau,"pre-send");
    impl_->arm_->setArmCmd(q,qd,tau);
    impl_->arm_->sendRecv(); // TX+RX; NEVER passive observation
    // Post-send observation check, still not an independent hardware watchdog.
    checkBenchCommand(*impl_->arm_,q,qd,tau,"post-send");
    const auto* l=impl_->arm_->lowstate;
    auto* comp=impl_->arm_->_ctrlComp;
    if(!l||!comp||!comp->udp||l->q.size()<6||l->dq.size()<6||l->tau.size()<6||
       l->temperature.size()<6||l->errorstate.size()<6||l->isMotorConnected.size()<6)
        throw std::runtime_error("SDK feedback absent/incomplete");
    Feedback f{};
    f.sdk_connected=!comp->udp->isDisConnect;
    f.lowcmd_reported=(comp->recvState.state==UNITREE_ARM::ArmFSMState::LOWCMD);
    f.read_at=Clock::now(); // sampled AFTER sendRecv(); not proof of fresh UDP reception!
    for(int i=0;i<6;i++){
        f.q[i]=l->q[i];f.dq[i]=l->dq[i];f.tau[i]=l->tau[i];
        f.temperature[i]=l->temperature[i]; f.motor_error[i]=static_cast<int>(l->errorstate[i] & static_cast<uint8_t>(~0x40u)); // SDK bit 0x40: Ignore
        f.motor_connection[i]=l->isMotorConnected[i];
    }
    return f;
#endif
}
}
