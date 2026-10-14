#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace b2z1::z1_jointctrl {

using Clock = std::chrono::steady_clock;
using Time = Clock::time_point;
using Joint6 = std::array<double, 6>;

struct Feedback {
    Joint6 q{};
    Joint6 dq{};
    Joint6 tau{};
    std::array<std::uint8_t, 6> motor_error{};
    std::array<std::uint8_t, 6> motor_connection{};
    Time received_at{};
    // MUST be supplied by a independently verified receive-path mechanism.
    // SDK lowstate values or isDisConnect alone cannot prove a new UDP datagram.
    bool confirmed_new_packet = false;
};

struct Config {
    Joint6 q_min{};
    Joint6 q_max{};
    Joint6 max_velocity_rad_s{};
    double max_tracking_error_rad = 0.20;
    std::chrono::milliseconds max_feedback_age{100};
    std::chrono::milliseconds max_command_interval{100};
    std::size_t min_consecutive_new_packets = 3;
};

struct JointCommand {
    Joint6 q{};
    Joint6 dq{};
};

// Logic only: NO vendor SDK, thread, socket, motor call or implicit mode switch.
// All limits must be explicitly supplied; there are no pretend 'real Z1' limits.
class SafetyCore final {
public:
    explicit SafetyCore(Config config);

    void receive(const Feedback& feedback, Time now);
    void authorize(bool electrical_inspection_confirmed,
                   bool operator_acknowledged,
                   bool sdk_jointctrl_ready,
                   Time now);
    void setGoal(const Joint6& desired_q, Time now);
    std::optional<JointCommand> next(Time now);
    void stop();

    bool faulted() const noexcept { return faulted_; }
    bool authorized() const noexcept { return authorized_; }
    const std::string& faultReason() const noexcept { return reason_; }
    std::size_t acceptedNewPackets() const noexcept { return packet_count_; }

private:
    [[noreturn]] void fail(const std::string& why);
    void requireFresh(Time now);
    Config config_;
    Feedback feedback_{};
    Joint6 commanded_q_{};
    Joint6 desired_q_{};
    Time last_command_at_{};
    bool has_feedback_ = false;
    bool faulted_ = false;
    bool authorized_ = false;
    bool goal_set_ = false;
    bool emitted_once_ = false;
    std::size_t packet_count_ = 0;
    std::string reason_;
};

} // namespace b2z1::z1_jointctrl
