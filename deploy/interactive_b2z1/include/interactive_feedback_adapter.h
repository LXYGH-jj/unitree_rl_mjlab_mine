#pragma once

#include "interactive_observation.h"

#include <array>
#include <chrono>

namespace interactive_b2z1 {

using MonoClock = std::chrono::steady_clock;
using MonoTime = MonoClock::time_point;
using Arm6 = std::array<float, 6>;

// Snapshot copied by the integration layer from B2 LowState and Z1 SDK.
// All timestamps MUST be receiver-side steady_clock timestamps from this host.
// Do not use robot system-clock, ROS time, or zero-initialized timestamps.
struct FeedbackSnapshot {
    Joint12 b2_motor_q{};   // SDK motor order: FL, FR, RL, RR
    Joint12 b2_motor_dq{};
    Arm6 z1_q{};            // Z1 joint1..joint6, from independent Z1 SDK
    Arm6 z1_dq{};
    std::array<float, 4> imu_quat_wxyz{1.f, 0.f, 0.f, 0.f};
    Command3 imu_gyro_xyz{};
    Command3 ee_goal_base_xyz{};  // desired TCP in B2 base frame (not actual TCP)
    Command3 base_command{};      // vx,vy,wz, unscaled
    MonoTime b2_received_at{};
    MonoTime z1_received_at{};
    bool b2_valid{false};
    bool z1_valid{false};
};

struct FeedbackAdapterConfig {
    std::chrono::milliseconds max_b2_age{100};
    std::chrono::milliseconds max_z1_age{100};
    std::chrono::milliseconds max_interstream_skew{50};
    // TRAINING nominal mount offset, not actual IK/mount location.
    Command3 nominal_arm_offset_yaw_frame{0.20f, 0.f, 0.09f};
};

struct AdaptedInput {
    RobotSample robot{};
    Command3 command{};
};

// Converts a desired TCP target expressed in robot-base coordinates into
// Interactive's training-frame EE goal observation. Actual Z1 mount geometry
// and FK/IK must be modeled separately.
Command3 eeGoalObservationFromBaseTarget(const Command3& goal_base,
                                         const std::array<float, 4>& quat_wxyz,
                                         const Command3& nominal_arm_offset_yaw_frame);

// Validates feedback before copying it into the 18-DOF policy order.
// Failure throws and MUST NOT result in sending a stale command.
// This is input validation only, not an actuator safety system.
AdaptedInput adaptFeedback(const FeedbackSnapshot& sample, MonoTime now,
                           const FeedbackAdapterConfig& cfg = {});

} // namespace interactive_b2z1
