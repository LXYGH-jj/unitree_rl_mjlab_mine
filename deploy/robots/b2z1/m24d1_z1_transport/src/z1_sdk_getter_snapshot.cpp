#include "z1_sdk_getter_snapshot.h"
#include <stdexcept>
namespace b2z1::m24d1 {
Snapshot copySdkGetterSnapshotUnverified(const UNITREE_ARM::unitreeArm& a,Time now) {
    Snapshot s{};
    // DO NOT invoke against a concurrently mutating live SDK object without a
    // verified SDK thread synchronization mechanism. Read is NOT fresh proof.
    const auto* p=a.lowstate;
    const auto* c=a._ctrlComp;
    if(!p||!c||!c->udp||p->q.size()<6||p->dq.size()<6||
       p->tau.size()<6||p->temperature.size()<6||
       p->errorstate.size()<6||p->isMotorConnected.size()<6)
        throw std::runtime_error("M2.4d1 SDK getter state incomplete");
    for(int i=0;i<6;++i) {
        s.q[i]=p->q[i];s.dq[i]=p->dq[i];s.tau[i]=p->tau[i];
        s.temperature_c[i]=p->temperature[i];
        s.motor_error[i]=p->errorstate[i];
        s.motor_connection[i]=p->isMotorConnected[i];
    }
    s.sdk_connected=!c->udp->isDisConnect;
    s.jointctrl_reported=c->recvState.state==UNITREE_ARM::ArmFSMState::JOINTCTRL;
    s.sampled_at=now; // ONLY observation time
    return s;
}
} // namespace b2z1::m24d1
