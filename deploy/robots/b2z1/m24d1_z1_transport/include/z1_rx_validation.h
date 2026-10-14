#pragma once
// M2.4d1: receiver-origin feedback validation; absolutely NO vendor IO.
#include "z1_jointctrl_safety.h"
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace b2z1::m24d1 {
using b2z1::z1_jointctrl::Joint6;
using b2z1::z1_jointctrl::Time;
using b2z1::z1_jointctrl::Feedback;

struct Snapshot {
    Joint6 q{}, dq{}, tau{};
    std::array<std::uint8_t,6> motor_error{}, motor_connection{};
    std::array<int,6> temperature_c{};
    bool sdk_connected = false;
    bool jointctrl_reported = false;
    Time sampled_at{}; // SDK getter time, NOT packet receive time
};

// This is an integration contract, not a self-certifying token. Only a real
// SDK receive-path hook that observes a COMPLETED UDP receive, validates peer,
// decodes *this very datagram* and timestamps the receive may populate it.
// Passing booleans here from polling/sendRecv cycles is PROHIBITED.
struct RxEvidence {
    std::uint64_t completed_receive_index = 0; // assigned by receive callback
    Time receive_completed_at{};
    bool callback_saw_completed_datagram = false;
    bool from_expected_peer = false;
    bool snapshot_decoded_from_this_datagram = false;
};

struct Limits {
    Joint6 q_min{}, q_max{};
    std::array<int,6> max_motor_temperature_c{}; // explicitly configured, not defaults
    std::chrono::milliseconds max_receive_age{100};
    std::chrono::milliseconds max_decode_lag{20};
};

class RxValidator final {
public:
    explicit RxValidator(Limits limits);
    // Getter-only snapshot ALWAYS ineligible; never make packet proof from time.
    std::optional<Feedback> fromSdkGetterOnly(const Snapshot& getter);
    // NOT a production packet tap; caller MUST independently audit the event's
    // origin. Offline tests are synthetic and do not qualify physical hardware.
    Feedback fromReceiverCallback(const Snapshot& decoded, const RxEvidence& event, Time now);
    bool faulted() const noexcept { return faulted_; }
    const std::string& reason() const noexcept { return reason_; }
    std::uint64_t acceptedPackets() const noexcept { return received_count_; }
private:
    [[noreturn]] void fail(const std::string& msg);
    Limits limits_;
    Time last_received_at_{};
    std::uint64_t last_index_ = 0;
    std::uint64_t received_count_ = 0;
    bool faulted_ = false;
    std::string reason_;
};
} // namespace b2z1::m24d1
