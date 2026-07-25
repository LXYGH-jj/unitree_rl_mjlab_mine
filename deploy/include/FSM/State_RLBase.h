// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <algorithm>

#include "FSMState.h"
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
#include "arm_command_manager.h"
#endif
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);
    
    void enter()
    {
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        stop_z1_diagnostic("enter Velocity");
#endif
        // set gain
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        const int body_joint_count = z1_sdk_enabled() ? 12 : static_cast<int>(env->robot->data.joint_stiffness.size());
#else
        const int body_joint_count = static_cast<int>(env->robot->data.joint_stiffness.size());
#endif
        for (int i = 0; i < body_joint_count; ++i)
        {
            lowcmd->msg_.motor_cmd()[i].kp() = env->robot->data.joint_stiffness[i];
            lowcmd->msg_.motor_cmd()[i].kd() = env->robot->data.joint_damping[i];
            lowcmd->msg_.motor_cmd()[i].dq() = 0;
            lowcmd->msg_.motor_cmd()[i].tau() = 0;
        }

#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        if (z1_sdk_enabled()) {
            const auto arm_cfg = env->cfg["commands"]["arm_pose"];
            auto& arm_manager = ArmCommandManager::instance();
            const auto arm_indices = arm_manager.zero_indices(arm_cfg);
            const auto arm_q = z1_sdk_controller->current_q(arm_manager.zero_q(arm_cfg, arm_indices.size()));
            z1_sdk_controller->update(arm_q);
            for (size_t i = 0; i < arm_indices.size() && i < arm_q.size(); ++i) {
                const int joint_id = arm_indices[i];
                lowcmd->msg_.motor_cmd()[joint_id].q() = arm_q[i];
                lowcmd->msg_.motor_cmd()[joint_id].kp() = 0.0f;
                lowcmd->msg_.motor_cmd()[joint_id].kd() = 0.0f;
                lowcmd->msg_.motor_cmd()[joint_id].dq() = 0.0f;
                lowcmd->msg_.motor_cmd()[joint_id].tau() = 0.0f;
            }
        }
#endif
        env->robot->update();
        reset_command_safety_targets();

        // Start policy thread
        policy_thread_running = true;
        policy_thread = std::thread([this]{
            using clock = std::chrono::high_resolution_clock;
            const std::chrono::duration<double> desiredDuration(env->step_dt);
            const auto dt = std::chrono::duration_cast<clock::duration>(desiredDuration);

            // Initialize timing
            auto sleepTill = clock::now() + dt;
            env->reset();

            while (policy_thread_running)
            {
                env->step();

                // Sleep
                std::this_thread::sleep_until(sleepTill);
                sleepTill += dt;
            }
        });
    }

    void run();
    
    void exit()
    {
        policy_thread_running = false;
        if (policy_thread.joinable()) {
            policy_thread.join();
        }

#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        if (z1_sdk_enabled()) {
            z1_sdk_controller->hold_current();
        }
#endif

        // Hold the current measured pose at the handoff boundary so the next
        // state does not inherit a stale policy target.
#ifdef UNITREE_RL_MJLAB_ENABLE_Z1_SDK
        const int hold_joint_count = z1_sdk_enabled() ? 12 : static_cast<int>(env->robot->data.joint_ids_map.size());
#else
        const int hold_joint_count = static_cast<int>(env->robot->data.joint_ids_map.size());
#endif
        for (int i = 0; i < hold_joint_count; ++i)
        {
            const int joint_id = env->robot->data.joint_ids_map[i];
            lowcmd->msg_.motor_cmd()[joint_id].q() = lowstate->msg_.motor_state()[joint_id].q();
            lowcmd->msg_.motor_cmd()[joint_id].dq() = 0;
            lowcmd->msg_.motor_cmd()[joint_id].tau() = 0;
        }
    }

private:
    void reset_command_safety_targets()
    {
        safe_joint_targets_.clear();
        const auto safety_cfg = env->cfg["command_safety"];
        max_target_delta_per_tick_ = 0.0f;
        if (safety_cfg && safety_cfg["max_target_delta_per_tick"] &&
            !safety_cfg["max_target_delta_per_tick"].IsNull())
        {
            max_target_delta_per_tick_ =
                std::max(0.0f, safety_cfg["max_target_delta_per_tick"].as<float>());
        }
        if (max_target_delta_per_tick_ <= 0.0f) {
            return;
        }

        safe_joint_targets_.resize(env->robot->data.joint_ids_map.size(), 0.0f);
        for (int i = 0; i < static_cast<int>(safe_joint_targets_.size()); ++i)
        {
            const int joint_id = env->robot->data.joint_ids_map[i];
            safe_joint_targets_[i] = lowstate->msg_.motor_state()[joint_id].q();
        }
    }

    float safe_joint_target(int index, float target)
    {
        if (max_target_delta_per_tick_ <= 0.0f ||
            index < 0 ||
            index >= static_cast<int>(safe_joint_targets_.size()))
        {
            return target;
        }

        auto& safe_target = safe_joint_targets_[index];
        safe_target = std::clamp(
            target,
            safe_target - max_target_delta_per_tick_,
            safe_target + max_target_delta_per_tick_
        );
        return safe_target;
    }

    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;

    std::thread policy_thread;
    bool policy_thread_running = false;
    std::vector<float> safe_joint_targets_;
    float max_target_delta_per_tick_ = 0.0f;
};

REGISTER_FSM(State_RLBase)
