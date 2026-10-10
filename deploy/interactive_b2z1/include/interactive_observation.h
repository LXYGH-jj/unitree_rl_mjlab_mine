#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace interactive_b2z1 {

constexpr std::size_t kLegDof = 12;
constexpr std::size_t kDof = 18;
constexpr std::size_t kProprioDim = 71;
constexpr std::size_t kPrivDim = 18;
constexpr std::size_t kHistoryLen = 10;
constexpr std::size_t kObsDim = kProprioDim * (kHistoryLen + 1) + kPrivDim; // 799

using Joint18 = std::array<float, kDof>;
using Joint12 = std::array<float, kLegDof>;
using Command3 = std::array<float, 3>; // vx, vy, wz
using Proprio71 = std::array<float, kProprioDim>;
using Observation799 = std::array<float, kObsDim>;

// SDK2 / MuJoCo motor order: FL, FR, RL, RR (three joints per leg).
// Interactive policy order: FR, FL, RR, RL.
constexpr std::array<std::size_t, kLegDof> kPolicyToB2Motor = {
    3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8
};

// Pure reordering; not an SDK2 interface and does not command motors.
Joint12 b2MotorToPolicy(const Joint12& motor_order);

struct ObservationConfig {
    Joint18 default_joint_pos{};
    Command3 cmd_scale{2.f, 2.f, 0.25f};
    float control_dt{0.02f};
    float gait_frequency_hz{2.f};
    float dof_pos_scale{1.f};
    float dof_vel_scale{0.05f};
    float ang_vel_scale{0.25f};
    float max_abs_action{100.f}; // Matches Python's observation history, NOT a motor safety limit.

    // Values copied from b2z1_interactive_sim2sim/config_lowlevel.yaml at baseline 1c48f7e.
    // Change this in lockstep with Python reference config, verified by parity test.
    static ObservationConfig baseline();
};

struct RobotSample {
    // Full 18 DOFs in Interactive policy order: FR FL RR RL, Z1 joint1..joint6.
    // Arm state MUST come from actual Z1 feedback in real-robot mode.
    Joint18 joint_q{};
    Joint18 joint_dq{};
    std::array<float, 4> imu_quat_wxyz{1.f, 0.f, 0.f, 0.f};
    Command3 imu_gyro_xyz{};
    // Desired EE goal in Interactive's *observation* frame, not measured TCP position:
    // R_base^T * (goal_world - [base_world + R_yaw * nominal_arm_base_offset]).
    Command3 ee_local_xyz{};
};

class ObservationBuilder final {
public:
    explicit ObservationBuilder(ObservationConfig config = ObservationConfig::baseline());

    // Resets gait, history, and last actor action. Call on each new episode.
    void reset();

    // Matches Python LowLevelWBC.make_obs() ordering and side effects:
    // concatenate current 71, priv 18, PREVIOUS 10x71 history; then update history.
    Observation799 build(const RobotSample& state, const Command3& velocity_command);

    // Call AFTER build() and actor inference; action becomes the next frame's last_action.
    // The first twelve values are used in proprio; all 18 values are stored/clipped.
    void acceptPolicyAction(const Joint18& raw_action);

    [[nodiscard]] const Proprio71& lastProprio() const noexcept { return last_proprio_; }
    [[nodiscard]] const Joint18& lastAction() const noexcept { return last_action_; }
    [[nodiscard]] std::uint64_t stepCount() const noexcept { return step_count_; }
    [[nodiscard]] double gaitPhase() const noexcept { return gait_phase_; }

private:
    ObservationConfig cfg_;
    double gait_phase_{0.0};
    std::uint64_t step_count_{0};
    Joint18 last_action_{};
    Proprio71 last_proprio_{};
    std::array<Proprio71, kHistoryLen> hist_{};
};

} // namespace interactive_b2z1
