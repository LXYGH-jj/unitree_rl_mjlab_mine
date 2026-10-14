#pragma once
// M2.4d5 simulation-only receiver-origin *synthetic tick* adapter.
// These ticks are NOT actual Z1 UDP packets. NO physical vendor transport.
#include "z1_transport_contract.h"
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace b2z1::m24d5 {
using Joint6 = b2z1::z1_jointctrl::Joint6;
using Time = b2z1::z1_jointctrl::Time;

class SimGate final {
public:
    SimGate(const Joint6& q_min, const Joint6& q_max,
            const std::string& cooperative_lock_file);
    SimGate(const SimGate&) = delete;
    SimGate& operator=(const SimGate&) = delete;
    ~SimGate();
    // Only called with atomic MuJoCo 20-slot LowState snapshots.
    // Returns the *mock-vetted* command, or nullopt during 3-tick warmup.
    std::optional<Joint6> step(std::uint32_t sim_tick,
                               const Joint6& measured_q,
                               const Joint6& measured_dq,
                               const Joint6& ik_goal,
                               Time now);
    void disarm() noexcept;
    bool faulted() const noexcept { return faulted_; }
    std::size_t commandsEmitted() const noexcept { return commands_; }
private:
    b2z1::m24d4::MockZ1Transport mock_;
    std::unique_ptr<b2z1::m24d4::MockOnlyRuntime> runtime_;
    std::uint32_t last_tick_{0};
    std::size_t new_ticks_{0}, commands_{0};
    bool have_tick_{false}, faulted_{false};
};
} // namespace b2z1::m24d5
