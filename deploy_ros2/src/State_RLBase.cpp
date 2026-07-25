#include "FSM/State_RLBase.h"

#include "go2_ros2/ros2_motor_access.h"
#include "go2_ros2/unitree_articulation_ros2.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/observations/observations.h"

State_RLBase::State_RLBase(int state_mode, std::string state_string)
: FSMState(state_mode, state_string)
{
    auto cfg = param::config["FSM"][state_string];
    auto policy_dir = param::parser_policy_dir(cfg["policy_dir"].as<std::string>());

    env = std::make_unique<isaaclab::ManagerBasedRLEnv>(
        YAML::LoadFile(policy_dir / "params" / "deploy.yaml"),
        std::make_shared<go2_ros2::Ros2BaseArticulation>(FSMState::lowstate));
    env->alg = std::make_unique<isaaclab::OrtRunner>(policy_dir / "exported" / "policy.onnx");

    registered_checks.emplace_back(
        std::make_pair(
            [&]() -> bool { return isaaclab::mdp::bad_orientation(env.get(), 1.0); },
            FSMStringMap.right.at("Passive")));
}

void State_RLBase::enter()
{
    for (size_t i = 0; i < env->robot->data.joint_stiffness.size(); ++i)
    {
        go2_ros2::motor::set_kp(*lowcmd, i, env->robot->data.joint_stiffness[i]);
        go2_ros2::motor::set_kd(*lowcmd, i, env->robot->data.joint_damping[i]);
        go2_ros2::motor::set_dq(*lowcmd, i, 0.0f);
        go2_ros2::motor::set_tau(*lowcmd, i, 0.0f);
    }

    env->robot->update();
    policy_thread_running = true;
    policy_thread = std::thread([this] {
        using clock = std::chrono::high_resolution_clock;
        const std::chrono::duration<double> desired_duration(env->step_dt);
        const auto dt = std::chrono::duration_cast<clock::duration>(desired_duration);
        auto sleep_till = clock::now() + dt;
        env->reset();

        while (policy_thread_running)
        {
            env->step();
            std::this_thread::sleep_until(sleep_till);
            sleep_till += dt;
        }
    });
}

void State_RLBase::run()
{
    auto action = env->action_manager->processed_actions();
    for (size_t i = 0; i < env->robot->data.joint_ids_map.size(); ++i)
    {
        const auto joint_id = static_cast<size_t>(env->robot->data.joint_ids_map[i]);
        go2_ros2::motor::set_q(*lowcmd, joint_id, action[i]);
    }
}

void State_RLBase::exit()
{
    policy_thread_running = false;
    if (policy_thread.joinable())
    {
        policy_thread.join();
    }
}

REGISTER_FSM(State_RLBase)
