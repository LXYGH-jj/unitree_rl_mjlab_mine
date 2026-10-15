#pragma once
// M2.4e2g: SDK2 DDS callback-side steady clock timestamp, NOT FSM first-observed time.
// Unitree SubscriptionBase calls post_communication() while mutex_ is held,
// after storing a newly received message. Access receivedAtLocked() only
// while the caller holds LowState::mutex_.
#include <unitree/dds_wrapper/robots/go2/go2_sub.h>
#include <chrono>

class StampedB2LowState final : public unitree::robot::go2::subscription::LowState {
public:
    using Clock = std::chrono::steady_clock;
    Clock::time_point receivedAtLocked() const noexcept { return received_at_; }
protected:
    void post_communication() override {
        received_at_ = Clock::now();
    }
private:
    Clock::time_point received_at_{};
};
