// M2.4e2b SDK adapter source. Compile-check ONLY. Not connected to a runnable worker.
// No recv timestamp provenance; feedback is explicitly getter sampled.
#include "z1_sdk_bridge.h"
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
    // SDK background thread used ONLY during mode transition, never during exchange().
};
SdkBridge::SdkBridge(bool has_gripper):impl_(std::make_unique<Impl>(has_gripper)){}
SdkBridge::~SdkBridge(){
    // NO backToStart and NO implicit PASSIVE, which can lead to dropped payloads.
    // Hardware shutdown behavior MUST be validated separately on a fixed bench.
    if(impl_->vendor_thread_running_ && impl_->arm_ && impl_->arm_->sendRecvThread){
        // Do not start a second send/receive owner on shutdown.
        impl_->arm_->sendRecvThread->shutdown();
        impl_->vendor_thread_running_=false;
    }
}
bool SdkBridge::started() const noexcept {return impl_->started_;}
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
    arm.setArmCmd(initial,::Vec6::Zero(),::Vec6::Zero());
    if(!arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD))
        throw std::runtime_error("LOWCMD transition failed");
    arm.sendRecvThread->shutdown();  // exclusive sendRecv owner after this point
    impl_->vendor_thread_running_=false;
    impl_->started_=true;
#endif
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
    for(int i=0;i<6;i++){q[i]=c.q[i];qd[i]=c.dq[i];tau[i]=c.tau[i];}
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
        f.temperature[i]=l->temperature[i]; f.motor_error[i]=l->errorstate[i];
        f.motor_connection[i]=l->isMotorConnected[i];
    }
    return f;
#endif
}
}
