#pragma once
#include "FSM/FSMState.h"
#include "interactive_pipeline.h"
#include "b2z1_live_input.h"
#include "z1_ipc_client.h"
#include "isaaclab/algorithms/algorithms.h"
#include <unitree_arm_sdk/model/ArmModel.h>
#include <chrono>
#include <memory>
#include <string>

// Real B2 DDS callback-stamped input, SDK/Mock Z1 IPC feedback, ONNX, IK.
// B2 motor writes require two explicit gates; Z1 SDK TARGET remains LOCKED
// in E2e client pending repaired-arm bench stop qualification.
class State_InteractiveArmReal final : public FSMState {
public:
    State_InteractiveArmReal(int, std::string);
    void enter() override;
    void run() override;
    void post_run() override;
    void exit() override;
private:
    using Clock=std::chrono::steady_clock;
    using Joint12=interactive_b2z1::Joint12;
    using Joint18=interactive_b2z1::Joint18;
    using Command3=interactive_b2z1::Command3;
    m24e2e::B2State captureB2() const;
    void checkAndUpdateIk(const m24e2e::ArmState&);
    void setFault(const std::string&) noexcept;
    void armKey(const std::string&, const m24e2e::ArmState&);
    static double maxAbs(const Vec6&);
    std::unique_ptr<isaaclab::OrtRunner> runner_;
    std::unique_ptr<interactive_b2z1::InteractivePipeline> pipeline_;
    m24e2e::ReadOnlyPoller z1_;
    UNITREE_ARM::Z1Model model_{};
    Vec6 arm_goal_q_=Vec6::Zero();
    HomoMat arm_entry_tool_=HomoMat::Identity();
    Eigen::Vector3d mount_=Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_xyz_=Eigen::Vector3d::Zero();
    Eigen::Vector3d origin_xyz_=Eigen::Vector3d::Zero();
    Joint12 b2_commanded_{},b2_desired_{};
    Command3 velocity_{0,0,0};
    Clock::time_point entered_{},next_motor_{},walk_started_{};
    std::string socket_path_{};
    std::string last_error_{};
    std::uint64_t motor_cycles_=0, policy_steps_=0;
    bool arm_initialized_=false, faulted_=false, send_b2_=false, output_pending_=false;
    bool walk_enabled_=false, walking_=false, motion_was_started_=false, walk_rearm_=false;
    bool use_ik_goal_=true;
    float b2_kp_=360.f,b2_kd_=5.f;
    float stand_step_=0.008f,walk_step_=0.04f;
    double tcp_offset_=0.0882;
    m24e2e::Mode mode_{m24e2e::Mode::SdkUnqualified};
};
REGISTER_FSM(State_InteractiveArmReal)
