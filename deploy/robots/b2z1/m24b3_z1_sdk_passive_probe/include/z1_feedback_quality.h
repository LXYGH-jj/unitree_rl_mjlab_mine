#pragma once
#include <array>
#include <chrono>
#include <cstdint>
#include <string>

namespace b2z1::m24b3 {
using Clock = std::chrono::steady_clock;
using Joint6 = std::array<double, 6>;

// A snapshot is NOT a verified new UDP datagram. In particular, sampled_at
// records the local observation time, not the network's receive timestamp.
struct Snapshot {
    Joint6 q{};
    Joint6 dq{};
    Joint6 tau{};
    std::array<std::uint8_t, 6> motor_error{};
    std::array<std::uint8_t, 6> motor_connection{};
    std::array<int, 6> temperature{};
    bool sdk_disconnected = true;
    bool sdk_state_passive = false;
    bool independently_confirmed_new_packet = false;
    Clock::time_point sampled_at{};
};

struct Quality {
    bool values_valid = false;
    bool motor_health_ok = false;
    bool sdk_reports_connected = false;
    bool controller_reports_passive = false;
    // Always false for this M2.4b3 SDK getter-only monitor.
    bool eligible_for_policy_feedback = false;
    std::string diagnostic;
};
Quality inspect(const Snapshot& s);
} // namespace b2z1::m24b3
