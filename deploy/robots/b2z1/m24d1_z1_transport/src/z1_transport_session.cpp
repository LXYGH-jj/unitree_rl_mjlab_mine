#include "z1_transport_session.h"
#include <exception>
#include <stdexcept>
#include <utility>
namespace b2z1::m24d1 {
TransportSession::TransportSession(b2z1::z1_jointctrl::Config cfg,Limits limits,
                                   b2z1::m24c3::CommandSink& sink):
    rx_(std::move(limits)),gate_(std::move(cfg),sink) {}
bool TransportSession::onSdkGetterOnly(const Snapshot& snap) {
    std::lock_guard<std::mutex> lock(mutex_);
    gate_.disarm(); // getter-only use must never keep motion enabled
    rx_.fromSdkGetterOnly(snap);
    // Fail closed for this session permanently: otherwise the previously
    // accumulated genuine RX count could be reused for reauthorization.
    receiver_fault_=true;
    receiver_reason_=rx_.reason();
    return false;
}
void TransportSession::onReceiverCallback(const Snapshot& s,const RxEvidence& e,Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(receiver_fault_) throw std::runtime_error("M2.4d1 receiver fault latched: "+receiver_reason_);
    try {
        auto fb=rx_.fromReceiverCallback(s,e,now);
        gate_.acceptFeedback(fb,now);
    }catch(const std::exception& ex){
        receiver_fault_=true;receiver_reason_=ex.what();gate_.disarm();throw;
    }
}
void TransportSession::authorize(bool el,bool op,bool sdk,Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(receiver_fault_) throw std::runtime_error("M2.4d1 receiver fault latched");
    gate_.authorize(el,op,sdk,now);
}
void TransportSession::setIkGoal(const Joint6& goal,Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(receiver_fault_) throw std::runtime_error("M2.4d1 receiver fault latched");
    gate_.updateIkGoal(goal,now);
}
bool TransportSession::tick(Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if(receiver_fault_) return false;
    return gate_.controlTick(now);
}
void TransportSession::disarm() {std::lock_guard<std::mutex> lock(mutex_);gate_.disarm();}
bool TransportSession::authorized() const {std::lock_guard<std::mutex> lock(mutex_);return !receiver_fault_&&gate_.authorized();}
bool TransportSession::faulted() const {std::lock_guard<std::mutex> lock(mutex_);return receiver_fault_||gate_.faulted();}
std::size_t TransportSession::sentCount() const {std::lock_guard<std::mutex> lock(mutex_);return gate_.sentCount();}
std::string TransportSession::faultReason() const {std::lock_guard<std::mutex> lock(mutex_);return receiver_fault_?receiver_reason_:gate_.failure();}
} // namespace b2z1::m24d1
