#include "FSM/State_RLBase.h"
#include "arm_command_manager.h"
#include "unitree_articulation.h"
#include "isaaclab/envs/mdp/observations/observations.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"

#include <array>
#include <iomanip>
#include <sstream>

namespace
{
void print_arm_state(
    LowCmd_t* lowcmd,
    LowState_t::SharedPtr lowstate,
    const std::shared_ptr<Z1SdkController>& z1_sdk_controller)
{
    static uint64_t last_print_ms = 0;
    const uint64_t now_ms = unitree::common::GetCurrentTimeMillisecond();
    if (now_ms - last_print_ms < 200) {
        return;
    }
    last_print_ms = now_ms;

    static constexpr std::array<int, 6> arm_ids = {12, 13, 14, 15, 16, 17};
    std::vector<float> arm_q(arm_ids.size(), 0.0f);
    std::vector<float> arm_dq(arm_ids.size(), 0.0f);
    std::vector<float> arm_tau(arm_ids.size(), 0.0f);
    std::vector<float> arm_q_cmd(arm_ids.size(), 0.0f);
    if (z1_sdk_controller && z1_sdk_controller->enabled() && z1_sdk_controller->has_state()) {
        arm_q = z1_sdk_controller->current_q(arm_q);
        arm_dq = z1_sdk_controller->current_dq(arm_ids.size());
        arm_tau = z1_sdk_controller->current_tau(arm_ids.size());
        arm_q_cmd = z1_sdk_controller->command_q(arm_ids.size());
    } else {
        for(size_t i = 0; i < arm_ids.size(); ++i) {
            arm_q[i] = lowstate->msg_.motor_state()[arm_ids[i]].q();
            arm_dq[i] = lowstate->msg_.motor_state()[arm_ids[i]].dq();
            arm_tau[i] = lowstate->msg_.motor_state()[arm_ids[i]].tau_est();
            arm_q_cmd[i] = lowcmd->msg_.motor_cmd()[arm_ids[i]].q();
        }
    }

    std::ostringstream os;
    os << std::fixed << std::setprecision(3);
    os << "Z1 arm ";

    os << "q=[";
    for(size_t i = 0; i < arm_ids.size(); ++i) {
        if(i) os << ",";
        os << arm_q[i];
    }
    os << "] dq=[";
    for(size_t i = 0; i < arm_ids.size(); ++i) {
        if(i) os << ",";
        os << arm_dq[i];
    }
    os << "] tau=[";
    for(size_t i = 0; i < arm_ids.size(); ++i) {
        if(i) os << ",";
        os << arm_tau[i];
    }
    os << "] q_cmd=[";
    for(size_t i = 0; i < arm_ids.size(); ++i) {
        if(i) os << ",";
        os << arm_q_cmd[i];
    }
    os << "] tau_cmd=[";
    for(size_t i = 0; i < arm_ids.size(); ++i) {
        if(i) os << ",";
        os << lowcmd->msg_.motor_cmd()[arm_ids[i]].tau();
    }
    os << "]";

    std::cout << os.str() << std::endl;
}
}


State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string) 
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<unitree::BaseArticulation<LowState_t::SharedPtr>>(FSMState::lowstate)
    );
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    this->registered_checks.emplace_back(
        std::make_pair(
            [&]()->bool{ return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")
        )
    );
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    const bool use_z1_sdk = FSMState::z1_sdk_enabled();
    const int body_joint_count = use_z1_sdk ? 12 : static_cast<int>(env->robot->data.joint_ids_map.size());
    for(int i(0); i < body_joint_count; i++) {
        const int joint_id = env->robot->data.joint_ids_map[i];
        lowcmd->msg_.motor_cmd()[joint_id].q() = action[i];
    }

    const auto arm_cfg = env->cfg["commands"]["arm_pose"];
    auto& arm_manager = ArmCommandManager::instance();
    const auto arm_indices = arm_manager.zero_indices(arm_cfg);
    auto current_q = arm_manager.zero_q(arm_cfg, arm_indices.size());
    if (use_z1_sdk) {
        current_q = z1_sdk_controller->current_q(current_q);
    } else {
        current_q.clear();
        current_q.reserve(arm_indices.size());
        for(int id : arm_indices) {
            if(id >= 0 && id < lowstate->msg_.motor_state().size()) {
                current_q.push_back(lowstate->msg_.motor_state()[id].q());
            } else {
                current_q.push_back(0.0f);
            }
        }
    }

    for(int id : arm_indices) {
        if(id >= 0 && id < lowcmd->msg_.motor_cmd().size()) {
            if (use_z1_sdk) {
                const auto it = std::find(arm_indices.begin(), arm_indices.end(), id);
                const size_t arm_offset = static_cast<size_t>(std::distance(arm_indices.begin(), it));
                if (arm_offset < current_q.size()) {
                    lowcmd->msg_.motor_cmd()[id].q() = current_q[arm_offset];
                }
            }
            if (use_z1_sdk) {
                lowcmd->msg_.motor_cmd()[id].kp() = 0.0f;
                lowcmd->msg_.motor_cmd()[id].kd() = 0.0f;
                lowcmd->msg_.motor_cmd()[id].dq() = 0.0f;
            }
            lowcmd->msg_.motor_cmd()[id].tau() = 0.0f;
        }
    }

    std::vector<float> final_arm_q = current_q;
    if(arm_manager.joint_trajectory_enabled(arm_cfg)) {
        final_arm_q = arm_manager.joint_command_q(arm_cfg, current_q);
        if (use_z1_sdk) {
            z1_sdk_controller->update(final_arm_q);
        } else {
            for(size_t i = 0; i < arm_indices.size() && i < final_arm_q.size(); ++i) {
                const int id = arm_indices[i];
                if(id >= 0 && id < lowcmd->msg_.motor_cmd().size()) {
                    lowcmd->msg_.motor_cmd()[id].q() = final_arm_q[i];
                }
            }
        }
        print_arm_state(lowcmd.get(), lowstate, z1_sdk_controller);
        return;
    }

    arm_manager.capture_policy_takeover_start(current_q);
    std::vector<float> arm_policy_q;
    arm_policy_q.reserve(arm_indices.size());
    for(int id : arm_indices) {
        const auto it = std::find(
            env->robot->data.joint_ids_map.begin(),
            env->robot->data.joint_ids_map.end(),
            id);
        if (it != env->robot->data.joint_ids_map.end()) {
            const size_t action_index = static_cast<size_t>(
                std::distance(env->robot->data.joint_ids_map.begin(), it));
            if (action_index < action.size()) {
                arm_policy_q.push_back(action[action_index]);
                continue;
            }
        }
        if(id >= 0 && id < lowcmd->msg_.motor_cmd().size()) {
            arm_policy_q.push_back(lowcmd->msg_.motor_cmd()[id].q());
        } else {
            arm_policy_q.push_back(0.0f);
        }
    }
    const auto blended_arm_q =
        arm_manager.blend_policy_takeover_q(arm_cfg, arm_policy_q, arm_indices.size());
    final_arm_q = blended_arm_q;

    if(arm_manager.should_override_arm_to_zero(arm_cfg)) {
        arm_manager.capture_zero_start(current_q);
        arm_manager.capture_off_hold(current_q);
        const auto start_q = arm_manager.zero_start_q(arm_indices.size());
        const auto target_q = arm_manager.target_q(arm_cfg, arm_indices.size());
        const float ratio = arm_manager.zero_ratio(arm_cfg);

        for(size_t i = 0; i < arm_indices.size(); ++i) {
            final_arm_q[i] = start_q[i] + ratio * (target_q[i] - start_q[i]);
        }
    }

    if (use_z1_sdk) {
        z1_sdk_controller->update(final_arm_q);
    } else {
        for(size_t i = 0; i < arm_indices.size() && i < final_arm_q.size(); ++i) {
            const int id = arm_indices[i];
            if(id >= 0 && id < lowcmd->msg_.motor_cmd().size()) {
                lowcmd->msg_.motor_cmd()[id].q() = final_arm_q[i];
            }
        }
    }

    print_arm_state(lowcmd.get(), lowstate, z1_sdk_controller);
}
