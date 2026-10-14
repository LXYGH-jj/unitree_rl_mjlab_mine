#include "z1_rx_validation.h"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace b2z1::m24d1 {
RxValidator::RxValidator(Limits limits): limits_(std::move(limits)) {
    if (limits_.max_receive_age.count()<=0 || limits_.max_decode_lag.count()<0)
        throw std::invalid_argument("M2.4d1 invalid timing limits");
    for(int i=0;i<6;++i) {
        if(!std::isfinite(limits_.q_min[i])||!std::isfinite(limits_.q_max[i])||
           limits_.q_min[i]>=limits_.q_max[i] ||
           limits_.max_motor_temperature_c[i]<1 ||
           limits_.max_motor_temperature_c[i]>150)
            throw std::invalid_argument("M2.4d1 requires reviewed joint/temperature limits");
    }
}
[[noreturn]] void RxValidator::fail(const std::string& msg) {
    faulted_=true; reason_=msg;
    throw std::runtime_error("M2.4d1 Z1 receiver proof fault: "+msg);
}
std::optional<Feedback> RxValidator::fromSdkGetterOnly(const Snapshot&) {
    // This MUST NEVER create a real feedback sample with confirmed_new_packet.
    reason_="SDK lowstate getter is NOT independently verified receive event";
    return std::nullopt;
}
Feedback RxValidator::fromReceiverCallback(const Snapshot& s,const RxEvidence& e,Time now) {
    if(faulted_) fail(reason_);
    if(!e.callback_saw_completed_datagram||!e.from_expected_peer||
       !e.snapshot_decoded_from_this_datagram)
        fail("no trusted, peer-checked, same-datagram receiver event");
    if(e.completed_receive_index==0 || e.completed_receive_index<=last_index_)
        fail("missing/replayed receive event index");
    if(e.receive_completed_at==Time{} || e.receive_completed_at>now ||
       now-e.receive_completed_at>limits_.max_receive_age ||
       (last_received_at_!=Time{} && e.receive_completed_at<=last_received_at_))
        fail("receive timestamp missing, stale or non-monotonic");
    if(s.sampled_at==Time{} || s.sampled_at<e.receive_completed_at ||
       s.sampled_at>now || s.sampled_at-e.receive_completed_at>limits_.max_decode_lag)
        fail("snapshot timestamp cannot be matched to receiver event");
    if(!s.sdk_connected||!s.jointctrl_reported)
        fail("SDK disconnected or JOINTCTRL not confirmed");
    Feedback fb{};
    for(int i=0;i<6;++i) {
        if(!std::isfinite(s.q[i])||!std::isfinite(s.dq[i])||!std::isfinite(s.tau[i])||
           s.q[i]<limits_.q_min[i]||s.q[i]>limits_.q_max[i]||
           s.motor_error[i]!=0 || s.motor_connection[i]!=0 ||
           s.temperature_c[i]<-40 || s.temperature_c[i]>limits_.max_motor_temperature_c[i])
            fail("nonfinite/out-of-range Z1 joint, motor error or overtemperature");
    }
    fb.q=s.q; fb.dq=s.dq; fb.tau=s.tau;
    fb.motor_error=s.motor_error; fb.motor_connection=s.motor_connection;
    fb.received_at=e.receive_completed_at;
    fb.confirmed_new_packet=true; // ONLY after matching to actual receive event
    last_index_=e.completed_receive_index;
    last_received_at_=e.receive_completed_at;
    ++received_count_;
    reason_.clear();
    return fb;
}
} // namespace b2z1::m24d1
