#pragma once
// M2.0a: Interactive simulator FSM state using the M1.5n SINGLE-LOOP execution order.
// Strictly SIM/lo only, never a physical robot controller.
#include "FSM/FSMState.h"
#include "interactive_pipeline.h"
#include "interactive_snapshot_store.h"
#include "isaaclab/algorithms/algorithms.h"
#include <unitree_arm_sdk/model/ArmModel.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

class State_InteractiveArmIK final : public FSMState {
public:
    State_InteractiveArmIK(int state, std::string state_string);
    ~State_InteractiveArmIK() = default;
    void enter() override;
    void run() override;
    void post_run() override;
    void exit() override;

private:
    using Clock = std::chrono::steady_clock;
    using Joint12 = interactive_b2z1::Joint12;
    using Arm6 = interactive_b2z1::Arm6;
    using Command3 = interactive_b2z1::Command3;

    struct SimSample {
        Joint12 leg_q{};
        Joint12 leg_dq{};
        Arm6 arm_q{};
        Arm6 arm_dq{};
        std::array<float, 4> quat{};
        Command3 gyro{};
        std::uint32_t tick_ms{0};
    };
    [[nodiscard]] SimSample read_sample() const;
    void process_keyboard(std::uint32_t tick_ms);
    void motor_step(const SimSample& sample, Clock::time_point now);
    void set_fault(const std::string& why) noexcept;
    void clear_motion();
    // Z1 is owned by IK in this FSM state; measured Z1 q/dq remain in 799D policy observation.
    void capture_arm_goal(const SimSample& s);
    void arm_key(const std::string& key);
    bool try_arm_goal(const Eigen::Vector3d& target);
    void write_motor_commands();
    void hold_on_policy_fault();
    static Vec6 as_vec6(const Arm6& v);
    static double max_abs(const Vec6& v);

    std::unique_ptr<isaaclab::OrtRunner> runner_;
    std::unique_ptr<interactive_b2z1::InteractivePipeline> pipeline_;
    interactive_b2z1::SnapshotStore snapshots_;
    Joint12 desired_{};
    Joint12 commanded_{};
    Arm6 arm_hold_{}; // entry reference, never a competing arm controller
    UNITREE_ARM::Z1Model arm_model_{};
    Vec6 arm_cmd_ = Vec6::Zero();
    Vec6 arm_goal_q_ = Vec6::Zero();
    HomoMat arm_entry_tool_ = HomoMat::Identity();
    Eigen::Vector3d mount_base_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d arm_goal_xyz_base_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d arm_initial_xyz_base_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d ee_baseline_xyz_ = Eigen::Vector3d(.45,0.,.35);
    double tcp_offset_x_{.0882};
    double position_step_m_{.01};
    double max_goal_shift_m_{.03};
    double max_arm_step_rad_{.004};
    double max_ik_joint_jump_rad_{.65};
    bool enable_xyz_keys_{false};
    bool enable_walking_{false};
    bool policy_ee_uses_arm_goal_{false};
    bool arm_motion_locked_{false};
    std::uint64_t rejected_goals_{0};
    Command3 velocity_cmd_{};
    Clock::time_point next_control_{};
    Clock::time_point last_sim_advance_{};
    std::uint32_t prev_sim_tick_{0};
    std::uint32_t enter_sim_tick_{0};
    std::uint32_t walk_start_tick_{0};
    std::uint64_t motor_steps_{0};
    std::uint64_t missed_deadlines_{0};
    bool publish_pending_{false};
    bool fault_{false};
    bool policy_ready_{false};
    bool walk_active_{false};
    bool walk_rearm_{false};
    bool has_walked_{false};

    float kp_{360.f}, kd_{5.f};
    float arm_kp_{90.f}, arm_kd_{6.f};
    float stand_step_{.008f}, walk_step_{.04f};
    float forward_vx_{.12f};
    std::uint32_t warmup_ms_{2000}, max_forward_ms_{2000};
};

REGISTER_FSM(State_InteractiveArmIK)
