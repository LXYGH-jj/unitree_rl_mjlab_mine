#pragma once
// M2.4e2b: offline-testable Z1 setpoint planner. No SDK, IO, motors.
#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
namespace m24e2b {
using Vec6=std::array<double,6>;
using Clock=std::chrono::steady_clock;
using Time=Clock::time_point;
struct Limits {
    Vec6 lo{}, hi{}, velocity_max{}, acceleration_max{};
    std::array<int,6> temperature_max{};
    std::chrono::milliseconds target_ttl{150};  // development value, NOT validated on hardware
};
struct Feedback {
    Vec6 q{}, dq{}, tau{};
    Vec6 sent_feedforward_tau{}; // Last SDK model+extra torque command, not motor feedback.
    std::array<int,6> temperature{}, motor_error{}, motor_connection{};
    bool sdk_connected{false};
    bool lowcmd_reported{false};
    Time read_at{}; // SDK getter sample time, NOT proven new UDP receive time.
};
struct Target { std::uint64_t seq{0}; Time issued_at{}; Vec6 q{}; bool enable{false}; };
struct Command { Vec6 q{}, dq{}, tau{}; bool track{false}; };
class Planner final {
public:
    explicit Planner(Limits);
    void reset(const Feedback&, Time);
    void acceptTarget(const Target&, Time);
    Command tick(const Feedback&, Time, double dt_seconds);
    void disarm() noexcept;
    bool faulted() const noexcept {return fault_;}
    const std::string& faultReason() const noexcept {return reason_;}
    bool armed() const noexcept {return armed_;}
    // Exact reference used for HOLD and the start of TRACK, independent of
    // feedback drift. May be outside strict target limits at initial startup;
    // arming must be denied until the physical reference is legal.
    Vec6 commandPosition() const noexcept {return q_command_;}
private:
    [[noreturn]] void reject(const char*);
    void validateFeedback(const Feedback&, Time);
    Limits limits_{};
    Vec6 q_command_{}, last_velocity_{};
    Target target_{};
    std::uint64_t last_sequence_{0};
    Time last_issue_{};
    bool initialized_{false}, armed_{false}, fault_{false};
    std::string reason_{};
};
}
