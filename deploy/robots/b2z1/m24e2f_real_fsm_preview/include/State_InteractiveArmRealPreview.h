#pragma once
// M2.4e2f: REAL INPUT PATH preview only, deliberately NO B2/Z1 motor command.
// Can be used only on MuJoCo loopback with the Z1 MOCK Worker in this revision.
#include "FSM/FSMState.h"
#include "interactive_pipeline.h"
#include "interactive_feedback_adapter.h"
#include "b2z1_live_input.h"
#include "z1_ipc_client.h"
#include "isaaclab/algorithms/algorithms.h"
#include <unitree_arm_sdk/model/ArmModel.h>
#include <array>
#include <chrono>
#include <memory>
#include <string>

class State_InteractiveArmRealPreview final : public FSMState {
public:
    State_InteractiveArmRealPreview(int state, std::string state_string);
    void enter() override;
    void run() override;
    void post_run() override;
    void exit() override;
private:
    using Clock=std::chrono::steady_clock;
    m24e2e::B2State captureB2(Clock::time_point now);
    void previewIk(const m24e2e::ArmState& measured);
    void fault(const std::string& reason) noexcept;
    std::unique_ptr<isaaclab::OrtRunner> runner_{};
    std::unique_ptr<interactive_b2z1::InteractivePipeline> pipeline_{};
    m24e2e::ReadOnlyPoller z1_{};
    UNITREE_ARM::Z1Model arm_model_{};
    Vec6 arm_goal_joint_=Vec6::Zero();
    HomoMat arm_entry_tool_=HomoMat::Identity();
    Eigen::Vector3d entry_goal_xyz_=Eigen::Vector3d::Zero();
    Eigen::Vector3d arm_goal_xyz_=Eigen::Vector3d::Zero();
    Eigen::Vector3d mount_=Eigen::Vector3d::Zero();
    std::array<float,3> fixed_goal_{.45f,0.f,.35f};
    std::array<float,3> cmd_{0.f,0.f,0.f};
    Clock::time_point next_control_{};
    Clock::time_point entered_{};
    Clock::time_point last_tick_first_seen_{};
    std::uint32_t last_tick_=0;
    std::uint64_t policy_steps_=0;
    double tcp_offset_x_=.0882;
    bool tick_seen_=false, ik_initialized_=false, faulted_=false;
    bool use_ik_ee_observation_=false;
    bool enable_ik_preview_keys_=false;
    std::string socket_path_{};
    std::string fault_reason_{};
};
REGISTER_FSM(State_InteractiveArmRealPreview)
