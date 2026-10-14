#pragma once
// Simulation-only joint-position command calculations; no DDS/SDK dependency.
#include "interactive_observation.h"
#include <array>
#include <cstddef>

namespace interactive_b2z1::sim_control {

struct TargetGuard {
    Joint12 q_target_policy{}; // FR,FL,RR,RL
    Joint12 q_target_motor{};  // FL,FR,RL,RR for THIS custom MuJoCo bridge
    std::size_t saturated{0};
};

// Action in FR FL RR RL order -> policy joint position targets -> SIM motor order.
// Always clamps to deliberately conservative SIM ranges. Not real-robot limits.
TargetGuard makeLegTargets(const Joint18& policy_action);

// Limit motion of the last commanded SIM joint target to max_step_rad per 5ms tick.
// Never invoke on a real robot: this is only a numerical safety helper.
Joint12 slewMotorTargets(const Joint12& desired, const Joint12& previous, float max_step_rad);

// Hard nonfinite, joint-range, angular-velocity, attitude checks on simulation feedback.
void checkSimFeedback(const Joint12& q_motor, const Joint12& dq_motor,
                      const std::array<float,4>& imu_wxyz);

} // namespace interactive_b2z1::sim_control
