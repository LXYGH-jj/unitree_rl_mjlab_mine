#pragma once

#include <algorithm>
#include <unordered_map>
#include <vector>

#include "isaaclab/envs/mdp/observations/observations.h"
#include "Types.h"

extern SportModeState_t::SharedPtr g_b2_sport_state;

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(b2_velocity_commands)
{
    struct CommandFilterState {
        std::vector<float> filtered = std::vector<float>(3, 0.0f);
        std::vector<float> output = std::vector<float>(3, 0.0f);
        long last_step = -1;
    };
    static std::unordered_map<ManagerBasedRLEnv*, CommandFilterState> command_filter_states;

    auto& state = command_filter_states[env];
    if(env->episode_length == 0 && state.last_step != 0) {
        std::fill(state.filtered.begin(), state.filtered.end(), 0.0f);
        std::fill(state.output.begin(), state.output.end(), 0.0f);
    }
    if(state.last_step == env->episode_length) {
        return state.output;
    }

    auto& joystick = env->robot->data.joystick;
    std::string command_name = "twist";
    if(params["command_name"]) {
        command_name = params["command_name"].as<std::string>();
    }

    const auto cfg = env->cfg["commands"][command_name]["ranges"];
    std::vector<float> target(3);
    target[0] = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    target[1] = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    target[2] = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    const auto command_cfg = env->cfg["commands"][command_name];
    const bool has_lin_acc = command_cfg["max_lin_acc"] && !command_cfg["max_lin_acc"].IsNull();
    const bool has_ang_acc = command_cfg["max_ang_acc"] && !command_cfg["max_ang_acc"].IsNull();

    if(has_lin_acc) {
        const float max_delta = command_cfg["max_lin_acc"].as<float>() * env->step_dt;
        for(int i = 0; i < 2; ++i) {
            const float delta = target[i] - state.filtered[i];
            state.filtered[i] += std::clamp(delta, -max_delta, max_delta);
        }
    } else {
        state.filtered[0] = target[0];
        state.filtered[1] = target[1];
    }

    if(has_ang_acc) {
        const float max_delta = command_cfg["max_ang_acc"].as<float>() * env->step_dt;
        const float delta = target[2] - state.filtered[2];
        state.filtered[2] += std::clamp(delta, -max_delta, max_delta);
    } else {
        state.filtered[2] = target[2];
    }

    state.output = state.filtered;
    state.last_step = env->episode_length;
    return state.output;
}

REGISTER_OBSERVATION(b2_gait_phase)
{
    float period = params["period"].as<float>();
    if (period <= 0.0f) {
        throw std::runtime_error("b2_gait_phase requires period > 0");
    }

    float delta_phase = env->step_dt * (1.0f / period);
    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    std::vector<float> command = b2_velocity_commands(env, params);
    float cmd_norm = std::sqrt(
        command[0] * command[0] +
        command[1] * command[1] +
        command[2] * command[2]
    );

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);

    if (cmd_norm < 0.1f) {
        obs[0] = 0.0f;
        obs[1] = 0.0f;
    }

    return obs;
}

}
}
