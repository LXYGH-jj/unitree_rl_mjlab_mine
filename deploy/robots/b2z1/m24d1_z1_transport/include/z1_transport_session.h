#pragma once
// Joins M2.4c3 safety session and M2.4d1 validated RX events. No Z1 IO.
#include "z1_jointctrl_session.h"
#include "z1_rx_validation.h"
#include <mutex>

namespace b2z1::m24d1 {
class TransportSession final {
public:
    TransportSession(b2z1::z1_jointctrl::Config safety,
                     Limits rx_limits,b2z1::m24c3::CommandSink& output);
    // Conservative SDK getter path disarms; it does not fabricate feedback.
    bool onSdkGetterOnly(const Snapshot& getter);
    // Only a genuine independently audited receive callback may call this in
    // future production code. This offline build has NO such SDK callback.
    void onReceiverCallback(const Snapshot& decoded,const RxEvidence& evidence,Time now);
    void authorize(bool electrical_inspected,bool operator_ack,bool sdk_jointctrl_ready,Time now);
    void setIkGoal(const Joint6& goal,Time now);
    bool tick(Time now);
    void disarm();
    bool authorized() const;
    bool faulted() const;
    std::size_t sentCount() const;
    std::string faultReason() const;
private:
    mutable std::mutex mutex_;
    RxValidator rx_;
    b2z1::m24c3::JointCtrlSession gate_;
    bool receiver_fault_=false;
    std::string receiver_reason_;
};
} // namespace b2z1::m24d1
