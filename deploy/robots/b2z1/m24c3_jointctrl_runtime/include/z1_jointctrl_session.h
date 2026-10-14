#pragma once
// M2.4c3: production-shaped command gate. This module has NO Z1 SDK,
// DDS or actuator imports. Only a separately reviewed sink can issue output.
#include "z1_jointctrl_safety.h"

#include <cstddef>
#include <mutex>
#include <string>

namespace b2z1::m24c3 {
using b2z1::z1_jointctrl::SafetyCore;
using b2z1::z1_jointctrl::Config;
using b2z1::z1_jointctrl::Feedback;
using b2z1::z1_jointctrl::Joint6;
using b2z1::z1_jointctrl::JointCommand;
using b2z1::z1_jointctrl::Time;

class CommandSink {
public:
    virtual ~CommandSink() = default;
    // Not a raw SDK command: safety must vet every emitted packet first.
    virtual void emit(const JointCommand& cmd) = 0;
};

// Construct only with explicit actual joint limits & safety config.
// Feed receiver-origin timestamps and TRUE independent new-UDP-packet proof.
// DO NOT infer freshness from lowstate getters, isDisConnect or loop iteration.
class JointCtrlSession final {
public:
    JointCtrlSession(Config config, CommandSink& sink);
    JointCtrlSession(const JointCtrlSession&) = delete;
    JointCtrlSession& operator=(const JointCtrlSession&) = delete;

    void acceptFeedback(const Feedback& feedback, Time now);
    void authorize(bool physical_electrical_check, bool attended_operator_ack,
                   bool vendor_jointctrl_confirmed, Time now);
    void updateIkGoal(const Joint6& target, Time now);
    // No command is emitted unless the gate is authorized and data are fresh.
    bool controlTick(Time now);
    // Software gate only; NOT an E-stop, PASSIVE command or gravity-safe hold.
    void disarm();

    bool authorized() const;
    bool faulted() const;
    std::size_t sentCount() const;
    std::string failure() const;

private:
    // SDK callbacks and the 200-Hz controller may be on different threads.
    // Serialize BOTH safety decisions and sends (one command owner).
    mutable std::mutex mutex_;
    SafetyCore guard_;
    CommandSink& sink_;
    std::size_t sends_{0};
    bool transport_fault_{false};
    std::string failure_;
};
} // namespace b2z1::m24c3
