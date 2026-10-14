#pragma once
// M2.4c2: SIM-ONLY FSM integration of the 799D observation and independent
// JOINTCTRL/IK command preview. No real SDK client, no JOINTCTRL mode switch.
#include "FSM/FSMState.h"
#include "m24c_shadow_bridge.h"
#include "isaaclab/algorithms/algorithms.h"
#include <unitree_arm_sdk/model/ArmModel.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>

class State_JointCtrlShadow final : public FSMState {
public:
    State_JointCtrlShadow(int state, std::string name);
    void enter() override;
    void run() override;
    void post_run() override;
    void exit() override;
private:
    using Clock = std::chrono::steady_clock;
    using J12 = interactive_b2z1::Joint12;
    using J6 = interactive_b2z1::Arm6;
    using C3 = interactive_b2z1::Command3;
    struct Sample {
        J12 legs{}, leg_dq{};
        J6 arm{}, arm_dq{};
        std::array<float,4> quat{};
        C3 gyro{};
        std::uint32_t tick{0};
    };
    Sample sample() const;
    void process_sim_keyboard();
    void update_ik_goal(const C3& proposal);
    void publish_sim_hold();
    void latch(const char* why) noexcept;
    std::unique_ptr<isaaclab::OrtRunner> actor_;
    std::unique_ptr<b2z1::m24c::ShadowBridge> shadow_;
    UNITREE_ARM::Z1Model model_{};
    J12 leg_hold_{};
    J6 arm_hold_{};
    J6 ik_preview_q_{};
    C3 goal_base_{};
    C3 entry_base_{};
    C3 mount_base_{.03f,0.f,.08f};
    double tcp_offset_x_{.0882};
    Clock::time_point next_eval_{};
    Clock::time_point last_sim_advance_{};
    std::uint32_t last_tick_{0};
    std::uint64_t frames_{0};
    bool entered_{false};
    bool fault_{false};
};

REGISTER_FSM(State_JointCtrlShadow)
