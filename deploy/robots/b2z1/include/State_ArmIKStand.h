#pragma once
// M1.9b: simulation-only native FSM state: B2 FixStand hold + vendor Z1Model IK.
// Never use on hardware. M1.8 InteractiveRL is intentionally untouched.
#include "FSM/FSMState.h"
#include <unitree_arm_sdk/model/ArmModel.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <string>

class State_ArmIKStand final : public FSMState {
public:
    State_ArmIKStand(int state, std::string state_string);
    void enter() override;
    void run() override;
    void post_run() override;
    void exit() override;
private:
    using Clock = std::chrono::steady_clock;
    using Vec3d = Eigen::Vector3d;
    struct Sample {
        std::array<float,12> leg_q{}, leg_dq{};
        Vec6 arm_q = Vec6::Zero(), arm_dq = Vec6::Zero();
        std::array<float,4> quat{};
        std::uint32_t tick{0};
    };
    Sample sample() const;
    void reset_goal_from_pose(const Vec6& q);
    void handle_key(const std::string& key, const Sample& s);
    bool try_goal(const Vec3d& next);
    static double maxabs(const Vec6& vec);
    bool guard_state(const Sample& s);
    void set_motor_commands();
    UNITREE_ARM::Z1Model model_{};
    Vec6 arm_cmd_ = Vec6::Zero();
    Vec6 arm_goal_q_ = Vec6::Zero();
    std::array<float,12> leg_hold_{};
    Vec3d goal_xyz_base_=Vec3d::Zero();
    Vec3d initial_xyz_base_=Vec3d::Zero();
    Vec3d mount_xyz_base_=Vec3d::Zero();
    HomoMat sdk_tool_pose_=HomoMat::Identity();
    double tcp_offset_sdk_x_{.0882};
    double position_step_m_{.01};
    double max_goal_shift_m_{.07};
    double max_arm_step_rad_{.004};
    double max_ik_joint_jump_rad_{.65};
    double leg_kp_{500.}, leg_kd_{5.}, arm_kp_{90.}, arm_kd_{6.};
    bool xyz_keys_enabled_{false};
    bool fault_latched_{false};
    bool publish_pending_{false};
    Clock::time_point next_control_{};
    Clock::time_point last_tick_wall_{};
    std::uint32_t prev_tick_{0};
    std::uint64_t control_steps_{0}, rejected_goals_{0};
};
REGISTER_FSM(State_ArmIKStand)
