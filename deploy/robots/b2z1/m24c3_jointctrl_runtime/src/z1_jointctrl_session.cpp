#include "z1_jointctrl_session.h"

#include <exception>
#include <stdexcept>

namespace b2z1::m24c3 {
JointCtrlSession::JointCtrlSession(Config cfg, CommandSink& sink)
    : guard_(cfg), sink_(sink) {}

void JointCtrlSession::acceptFeedback(const Feedback& feedback, Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transport_fault_) throw std::runtime_error("M2.4c3 sink fault latched: " + failure_);
    guard_.receive(feedback, now);
}

void JointCtrlSession::authorize(bool electrical, bool operator_ack,
                                 bool jointctrl_ready, Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transport_fault_) throw std::runtime_error("M2.4c3 sink fault latched");
    guard_.authorize(electrical, operator_ack, jointctrl_ready, now);
}

void JointCtrlSession::updateIkGoal(const Joint6& goal, Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transport_fault_) throw std::runtime_error("M2.4c3 sink fault latched");
    guard_.setGoal(goal, now);
}

bool JointCtrlSession::controlTick(Time now) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (transport_fault_) throw std::runtime_error("M2.4c3 sink fault latched: " + failure_);
    const auto command = guard_.next(now);
    if (!command) return false;
    try {
        sink_.emit(*command);
    } catch (const std::exception& ex) {
        transport_fault_ = true;
        failure_ = ex.what();
        guard_.stop();
        throw std::runtime_error("M2.4c3 sink error, software output disabled: " + failure_);
    } catch (...) {
        transport_fault_ = true;
        failure_ = "unknown sink error";
        guard_.stop();
        throw std::runtime_error("M2.4c3 sink error, software output disabled");
    }
    ++sends_;
    return true;
}

void JointCtrlSession::disarm() {
    std::lock_guard<std::mutex> lock(mutex_);
    guard_.stop();
}
bool JointCtrlSession::authorized() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return !transport_fault_ && guard_.authorized();
}
bool JointCtrlSession::faulted() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return transport_fault_ || guard_.faulted();
}
std::size_t JointCtrlSession::sentCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sends_;
}
std::string JointCtrlSession::failure() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return transport_fault_ ? failure_ : guard_.faultReason();
}
} // namespace b2z1::m24c3
