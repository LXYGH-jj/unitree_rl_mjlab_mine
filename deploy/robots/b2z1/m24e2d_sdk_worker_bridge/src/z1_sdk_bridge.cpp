// M2.4e2h: Z1 SDK bridge for standalone worker; PHYSICAL I/O requires supervised bench.
// Hardware start(), exchange() both affect motors. NEVER treat as read-only.
// No recv timestamp provenance; feedback is explicitly getter sampled.
#include "z1_sdk_bridge.h"
#include "m24e2h_gravity_guard.h"
#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/arm_common.h>
#include <stdexcept>
#include <cmath>
#include <thread>
namespace m24e2b {
class SdkBridge::Impl {
public:
    explicit Impl(bool with_gripper):with_gripper_(with_gripper){}
    bool with_gripper_{false};
    bool started_{false};
    bool vendor_thread_running_{false};
    std::unique_ptr<UNITREE_ARM::unitreeArm> arm_;
    std::thread::id owner_{};
    Feedback initial_{};
    // SDK background thread used ONLY during mode transition, never during exchange().
};
SdkBridge::SdkBridge(bool has_gripper):impl_(std::make_unique<Impl>(has_gripper)){}
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
    if(!impl_->arm_) return false;
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
    // Match the official C++ LOWCMD example: retain the vendor-configured gains.
    // Never silently replace them with ALORE Python wrapper's differently scaled gains.
    if(!arm.lowcmd || arm.lowcmd->kp.size()<6 || arm.lowcmd->kd.size()<6)
        throw std::runtime_error("Z1 LOWCMD PD gains unavailable");
    for(int i=0;i<6;++i) {
        if(!std::isfinite(arm.lowcmd->kp[i]) || !std::isfinite(arm.lowcmd->kd[i]) ||
           arm.lowcmd->kp[i]<=0 || arm.lowcmd->kd[i]<=0)
            throw std::runtime_error("Z1 LOWCMD PD gains nonpositive/nonfinite");
    }
    ::Vec6 model_tau=arm._ctrlComp->armModel->inverseDynamics(
        initial, ::Vec6::Zero(), ::Vec6::Zero(), ::Vec6::Zero());
    m24e2h::Vec6 measured_g{}, user_tau{};
    for(int i=0;i<6;++i)measured_g[i]=model_tau[i];
    const auto guarded=m24e2h::combineGravityTorque(measured_g,user_tau);
    ::Vec6 compensated=::Vec6::Zero();
    for(int i=0;i<6;++i)compensated[i]=guarded[i];
    arm.setArmCmd(initial,::Vec6::Zero(),compensated);
    if(!arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD))
        throw std::runtime_error("LOWCMD transition failed");
    arm.sendRecvThread->shutdown();  // exclusive sendRecv owner after this point
    impl_->vendor_thread_running_=false;
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
    impl_->arm_->setArmCmd(q,qd,tau);
    impl_->arm_->sendRecv(); // TX+RX; NEVER passive observation
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
