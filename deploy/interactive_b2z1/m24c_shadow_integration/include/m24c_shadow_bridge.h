#pragma once
// M2.4c1: COMPOSITION / REPLAY ONLY. No SDK, network or motor command types.
#include "interactive_pipeline.h"
#include "interactive_snapshot_store.h"
#include <array>
#include <stdexcept>
#include <string>

namespace b2z1::m24c {
using namespace interactive_b2z1;
enum class Mode { Stand, Walk };

struct Preview {
    Observation799 observation{};
    Joint18 policy_action{}; // raw/clipped policy action, not a motor target
    Joint12 leg_policy_action{}; // FIRST 12 in training order (FR FL RR RL)
    Arm6 z1_ik_goal_unqualified{}; // separate IK goal, never from actor arm 6
    Command3 ee_obs_xyz{};
    // There is intentionally no actuator-ready command and no hardware API.
};

// Assembles the actual 799D adapter + history + policy split with a mock/injected
// actor. It cannot send Z1 joint commands or B2 LowCmd under any circumstances.
class ShadowBridge final {
public:
    explicit ShadowBridge(PolicyEvaluator actor);
    void onB2(const Joint12& q_motor, const Joint12& dq_motor,
              const std::array<float,4>& imu_wxyz, const Command3& gyro,
              MonoTime reception_timestamp);
    // confirmed_new_packet is only a MOCK test input in M2.4c1. Do not pass
    // true in production without receive-event verification at Z1 UDP boundary.
    void onZ1(const Arm6& q, const Arm6& dq, MonoTime reception_timestamp,
              bool independently_confirmed_new_packet);
    Preview preview(Mode mode, const Command3& base_command,
                    const Command3& entry_ee_base, const Command3& goal_ee_base,
                    const Arm6& ik_q_unqualified, MonoTime now);
    void resetOffline();
    bool faulted() const noexcept { return faulted_; }
    const std::string& faultReason() const noexcept { return reason_; }
private:
    [[noreturn]] void fault(const std::string& reason);
    SnapshotStore store_;
    InteractivePipeline pipeline_;
    bool confirmed_z1_ = false;
    bool faulted_ = false;
    std::string reason_;
};
} // namespace b2z1::m24c
