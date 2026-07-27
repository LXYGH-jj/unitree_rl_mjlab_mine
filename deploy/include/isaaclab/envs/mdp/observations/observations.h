// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <array>
#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "arm_command_manager.h"
#include "isaaclab/envs/manager_based_rl_env.h"
#include "isaaclab/sensors/depth_frame_provider.h"
#include "isaaclab/sensors/depth_preprocess.h"

namespace isaaclab
{
namespace mdp
{

inline std::vector<float> twist_commands(ManagerBasedRLEnv* env, YAML::Node params);
inline std::vector<float> gait_phase(ManagerBasedRLEnv* env, YAML::Node params);

inline float wrap_to_pi(float angle)
{
    while (angle > static_cast<float>(M_PI)) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

inline float yaw_from_quat(const Eigen::Quaternionf& q)
{
    return std::atan2(
        2.0f * (q.w() * q.z() + q.x() * q.y()),
        1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z())
    );
}

inline std::pair<size_t, size_t> depth_zero_over20_counts(const std::vector<float>& values)
{
    size_t zero_count = 0;
    size_t over20_count = 0;
    for (const float value : values) {
        if (std::isfinite(value)) {
            if (value == 0.0f) {
                ++zero_count;
            }
            if (value > 20.0f) {
                ++over20_count;
            }
        }
    }
    return {zero_count, over20_count};
}

inline std::pair<float, float> depth_student_waypoint_yaws(
    ManagerBasedRLEnv* env,
    YAML::Node params)
{
    if (!params || !params["waypoints"] || params["waypoints"].IsNull()) {
        return {0.0f, 0.0f};
    }
    const auto wp_cfg = params["waypoints"];
    if (!wp_cfg["enabled"].as<bool>(false) || !wp_cfg["points"] || !wp_cfg["points"].IsSequence()) {
        return {0.0f, 0.0f};
    }

    std::vector<std::pair<float, float>> points;
    for (const auto& point : wp_cfg["points"]) {
        if (!point.IsSequence() || point.size() < 2) {
            continue;
        }
        points.emplace_back(point[0].as<float>(), point[1].as<float>());
    }
    if (points.empty()) {
        return {0.0f, 0.0f};
    }

    struct WaypointState {
        bool initialized = false;
        long last_episode_length = -1;
        int index = 0;
        int reach_timer = 0;
        float x = 0.0f;
        float y = 0.0f;
    };
    static WaypointState state;

    const float yaw = yaw_from_quat(env->robot->data.root_quat_w);
    float origin_x = 0.0f;
    float origin_y = 0.0f;
    if (wp_cfg["origin"] && wp_cfg["origin"].IsSequence() && wp_cfg["origin"].size() >= 2) {
        origin_x = wp_cfg["origin"][0].as<float>();
        origin_y = wp_cfg["origin"][1].as<float>();
    }
    const bool use_root_pos_w =
        wp_cfg["use_root_pos_w"] && !wp_cfg["use_root_pos_w"].IsNull()
            ? wp_cfg["use_root_pos_w"].as<bool>()
            : false;
    const bool has_root_pos_w = use_root_pos_w && env->robot->data.root_pos_w_valid;
    if (!state.initialized || env->episode_length <= 1 || env->episode_length <= state.last_episode_length) {
        state = WaypointState{};
        state.initialized = true;
        state.last_episode_length = env->episode_length;
        if (has_root_pos_w) {
            state.x = env->robot->data.root_pos_w.x() - origin_x;
            state.y = env->robot->data.root_pos_w.y() - origin_y;
        }
    } else if (has_root_pos_w) {
        state.x = env->robot->data.root_pos_w.x() - origin_x;
        state.y = env->robot->data.root_pos_w.y() - origin_y;
        state.last_episode_length = env->episode_length;
    } else {
        const auto& vel_b = env->robot->data.root_lin_vel_b;
        const float cos_yaw = std::cos(yaw);
        const float sin_yaw = std::sin(yaw);
        const float vx_w = cos_yaw * vel_b.x() - sin_yaw * vel_b.y();
        const float vy_w = sin_yaw * vel_b.x() + cos_yaw * vel_b.y();
        state.x += vx_w * env->step_dt;
        state.y += vy_w * env->step_dt;
        state.last_episode_length = env->episode_length;
    }

    static int print_count = 0;
    if (++print_count % 50 == 0) {
        std::cout
            << "[waypoint_pos] use_root_pos_w=" << use_root_pos_w
            << " valid=" << env->robot->data.root_pos_w_valid
            << " state_xy=" << state.x << ", " << state.y
            << " root_pos_w="
            << env->robot->data.root_pos_w.x() << ", "
            << env->robot->data.root_pos_w.y()
            << " origin=" << origin_x << ", " << origin_y
            << " idx=" << state.index
            << std::endl;
    }

    const float reach_radius =
        wp_cfg["reach_radius"] && !wp_cfg["reach_radius"].IsNull()
            ? wp_cfg["reach_radius"].as<float>()
            : 0.35f;
    const float reach_delay =
        wp_cfg["reach_delay"] && !wp_cfg["reach_delay"].IsNull()
            ? wp_cfg["reach_delay"].as<float>()
            : 0.1f;
    const int delay_steps = static_cast<int>(reach_delay / env->step_dt);
    if (state.reach_timer > delay_steps && state.index + 1 < static_cast<int>(points.size())) {
        state.index += 1;
        state.reach_timer = 0;
    }
    if (state.index + 1 < static_cast<int>(points.size())) {
        const auto& goal = points[state.index];
        const float dx = goal.first - state.x;
        const float dy = goal.second - state.y;
        if (std::sqrt(dx * dx + dy * dy) < reach_radius) {
            state.reach_timer += 1;
        }
    }

    auto clamped_goal = [&](const int idx) -> const std::pair<float, float>& {
        const int clamped_idx = std::clamp(idx, 0, static_cast<int>(points.size()) - 1);
        return points[clamped_idx];
    };
    auto goal_yaw = [&](const int idx) {
        const auto& goal = clamped_goal(idx);
        return wrap_to_pi(std::atan2(goal.second - state.y, goal.first - state.x) - yaw);
    };
    const float delta_yaw = goal_yaw(state.index);
    const float delta_next_yaw = goal_yaw(state.index + 1);

    const auto& target = clamped_goal(state.index);
    const auto& next_target = clamped_goal(state.index + 1);
    const float target_dx = target.first - state.x;
    const float target_dy = target.second - state.y;
    const float next_dx = next_target.first - state.x;
    const float next_dy = next_target.second - state.y;
    env->debug_telemetry.origin_x = origin_x;
    env->debug_telemetry.origin_y = origin_y;
    env->debug_telemetry.progress_x = state.x;
    env->debug_telemetry.robot_yaw = yaw;
    env->debug_telemetry.waypoint_idx = state.index;
    env->debug_telemetry.reach_timer = state.reach_timer;
    env->debug_telemetry.waypoint_state_x = state.x;
    env->debug_telemetry.waypoint_state_y = state.y;
    env->debug_telemetry.target_x = target.first;
    env->debug_telemetry.target_y = target.second;
    env->debug_telemetry.next_x = next_target.first;
    env->debug_telemetry.next_y = next_target.second;
    env->debug_telemetry.dist_to_target = std::sqrt(target_dx * target_dx + target_dy * target_dy);
    env->debug_telemetry.dist_to_next = std::sqrt(next_dx * next_dx + next_dy * next_dy);
    env->debug_telemetry.waypoint_delta_yaw = delta_yaw;
    env->debug_telemetry.waypoint_delta_next_yaw = delta_next_yaw;
    return {delta_yaw, delta_next_yaw};
}

inline std::pair<float, float> depth_student_command_yaws(
    ManagerBasedRLEnv* env,
    YAML::Node params)
{
    const auto waypoint_yaws = isaaclab::mdp::depth_student_waypoint_yaws(env, params);
    if (params && params["waypoints"] && !params["waypoints"].IsNull()) {
        const auto wp_cfg = params["waypoints"];
        if (wp_cfg["enabled"].as<bool>(false)) {
            return waypoint_yaws;
        }
    }

    if (params && params["yaw_input"] && !params["yaw_input"].IsNull()) {
        const auto yaw_cfg = params["yaw_input"];
        if (yaw_cfg["enabled"].as<bool>(false)) {
            const std::string source =
                yaw_cfg["source"] && !yaw_cfg["source"].IsNull()
                    ? yaw_cfg["source"].as<std::string>()
                    : "joystick_rx";
            auto& joystick = env->robot->data.joystick;
            float value = 0.0f;
            if (joystick != nullptr) {
                if (source == "joystick_rx") {
                    value = -joystick->rx();
                } else if (source == "joystick_lx") {
                    value = -joystick->lx();
                }
            }

            const float deadzone =
                yaw_cfg["deadzone"] && !yaw_cfg["deadzone"].IsNull()
                    ? std::max(0.0f, yaw_cfg["deadzone"].as<float>())
                    : 0.05f;
            if (std::abs(value) < deadzone) {
                value = 0.0f;
            }
            const float scale =
                yaw_cfg["scale"] && !yaw_cfg["scale"].IsNull()
                    ? yaw_cfg["scale"].as<float>()
                    : 0.6f;
            const float next_scale =
                yaw_cfg["next_scale"] && !yaw_cfg["next_scale"].IsNull()
                    ? yaw_cfg["next_scale"].as<float>()
                    : scale;
            return {value * scale, value * next_scale};
        }
    }

    return waypoint_yaws;
}

inline float depth_student_forward_speed(
    ManagerBasedRLEnv* env,
    YAML::Node params,
    YAML::Node command_cfg)
{
    float forward_speed = 0.0f;
    if (params && params["waypoints"] && !params["waypoints"].IsNull()) {
        const auto wp_cfg = params["waypoints"];
        if (wp_cfg["enabled"].as<bool>(false) &&
            wp_cfg["forward_speed"] && !wp_cfg["forward_speed"].IsNull()) {
            forward_speed = wp_cfg["forward_speed"].as<float>();
        }
    }

    if (forward_speed == 0.0f) {
        auto & joystick = env->robot->data.joystick;
        if (joystick != nullptr) {
            forward_speed = joystick->ly();
        }
    }

    if (command_cfg && !command_cfg.IsNull()) {
        forward_speed = std::clamp(
            forward_speed,
            command_cfg["lin_vel_x"][0].as<float>(),
            command_cfg["lin_vel_x"][1].as<float>()
        );
    }
    return forward_speed;
}

REGISTER_OBSERVATION(base_ang_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(base_angular_velocity)
{
    return isaaclab::mdp::base_ang_vel(env, params);
}

REGISTER_OBSERVATION(base_linear_velocity)
{
    if (params && params["command_name"]) {
        return isaaclab::mdp::twist_commands(env, params);
    }
    if (env->cfg["commands"] && env->cfg["commands"]["arm_pose"]) {
        YAML::Node twist_params;
        twist_params["command_name"] = "twist";
        return isaaclab::mdp::twist_commands(env, twist_params);
    }

    auto & asset = env->robot;
    auto & data = asset->data.root_lin_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    auto & asset = env->robot;
    std::vector<float> data;

    std::vector<int> joint_ids;
    try {
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
    } catch(const std::exception& e) {
    }

    if(joint_ids.empty())
    {
        data.resize(asset->data.joint_pos.size());
        for(size_t i = 0; i < asset->data.joint_pos.size(); ++i)
        {
            data[i] = asset->data.joint_pos[i];
        }
    }
    else
    {
        data.resize(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i)
        {
            data[i] = asset->data.joint_pos[joint_ids[i]];
        }
    }

    return data;
}

REGISTER_OBSERVATION(joint_pos_rel)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    try {
        std::vector<int> joint_ids;
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
        if(!joint_ids.empty()) {
            std::vector<float> tmp_data;
            tmp_data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i){
                tmp_data[i] = data[joint_ids[i]];
            }
            data = tmp_data;
        }
    } catch(const std::exception& e) {
    
    }

    return data;
}

REGISTER_OBSERVATION(joint_positions)
{
    return isaaclab::mdp::joint_pos_rel(env, params);
}

REGISTER_OBSERVATION(joint_vel_rel)
{
    auto & asset = env->robot;
    auto data = asset->data.joint_vel;

    try {
        const std::vector<int> joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();

        if(!joint_ids.empty()) {
            data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i) {
                data[i] = asset->data.joint_vel[joint_ids[i]];
            }
        }
    } catch(const std::exception& e) {
    }
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_velocities)
{
    return isaaclab::mdp::joint_vel_rel(env, params);
}

REGISTER_OBSERVATION(last_action)
{
    auto data = env->action_manager->action();
    return std::vector<float>(data.data(), data.data() + data.size());
};

REGISTER_OBSERVATION(actions)
{
    return isaaclab::mdp::last_action(env, params);
}

REGISTER_OBSERVATION(velocity_commands)
{
    std::vector<float> obs(3);
    auto & joystick = env->robot->data.joystick;

    std::string command_name = "base_velocity";
    if(params["command_name"])
    {
        command_name = params["command_name"].as<std::string>();
    }
    const auto cfg = env->cfg["commands"][command_name]["ranges"];

    obs[0] = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    return obs;
}

REGISTER_OBSERVATION(commands)
{
    return isaaclab::mdp::twist_commands(env, params);
}

REGISTER_OBSERVATION(twist_commands)
{
    std::vector<float> obs(3, 0.0f);
    auto & joystick = env->robot->data.joystick;
    if (joystick == nullptr) {
        return obs;
    }

    std::string command_name = "twist";
    if(params && params["command_name"] && !params["command_name"].IsNull()) {
        command_name = params["command_name"].as<std::string>();
    }
    const auto cfg = env->cfg["commands"][command_name]["ranges"];
    if (!cfg || cfg.IsNull()) {
        return obs;
    }

    const bool no_input =
        std::abs(joystick->ly()) < 1.0e-3f &&
        std::abs(joystick->lx()) < 1.0e-3f &&
        std::abs(joystick->rx()) < 1.0e-3f;
    if (no_input) {
        return obs;
    }

    obs[0] = std::clamp(joystick->ly(), cfg["lin_vel_x"][0].as<float>(), cfg["lin_vel_x"][1].as<float>());
    obs[1] = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    return obs;
}

REGISTER_OBSERVATION(arm_pose_command)
{
    return ArmCommandManager::instance().pose_command(
        env->cfg["commands"]["arm_pose"],
        env->robot->data.joystick
    );
}

REGISTER_OBSERVATION(foot_contacts_heuristic)
{
    std::vector<float> contacts(4, 0.0f);
    const auto & q = env->robot->data.joint_pos;
    const auto & dq = env->robot->data.joint_vel;
    if (q.size() < 12 || dq.size() < 12) {
        return contacts;
    }

    float calf_pos_threshold = -1.0f;
    float calf_vel_threshold = 1.5f;
    if (params && params["calf_pos_threshold"] && !params["calf_pos_threshold"].IsNull()) {
        calf_pos_threshold = params["calf_pos_threshold"].as<float>();
    }
    if (params && params["calf_vel_threshold"] && !params["calf_vel_threshold"].IsNull()) {
        calf_vel_threshold = params["calf_vel_threshold"].as<float>();
    }

    static constexpr std::array<int, 4> calf_ids = {2, 5, 8, 11};
    for (size_t i = 0; i < calf_ids.size(); ++i) {
        const int id = calf_ids[i];
        const bool likely_contact =
            (q[id] < calf_pos_threshold) && (std::abs(dq[id]) < calf_vel_threshold);
        contacts[i] = likely_contact ? 1.0f : 0.0f;
    }

    return contacts;
}

REGISTER_OBSERVATION(foot_contacts_from_force)
{
    if (params && params["foot_contact_override"] && params["foot_contact_override"].IsSequence()) {
        const auto override_contacts = params["foot_contact_override"].as<std::vector<float>>();
        std::vector<float> contacts(4, 0.0f);
        for (size_t i = 0; i < contacts.size() && i < override_contacts.size(); ++i) {
            contacts[i] = override_contacts[i] > 0.5f ? 1.0f : 0.0f;
        }
        return contacts;
    }

    if (!env->robot->data.foot_force_available) {
        return isaaclab::mdp::foot_contacts_heuristic(env, params);
    }

    std::string source = "foot_force_est";
    if (params && params["foot_force_source"] && !params["foot_force_source"].IsNull()) {
        source = params["foot_force_source"].as<std::string>();
    }

    float threshold = 20.0f;
    if (params && params["foot_force_threshold"] && !params["foot_force_threshold"].IsNull()) {
        threshold = params["foot_force_threshold"].as<float>();
    }

    bool use_abs = false;
    if (params && params["foot_force_use_abs"] && !params["foot_force_use_abs"].IsNull()) {
        use_abs = params["foot_force_use_abs"].as<bool>();
    }

    std::vector<float> bias(4, 0.0f);
    if (params && params["foot_force_bias"] && !params["foot_force_bias"].IsNull()) {
        const auto cfg_bias = params["foot_force_bias"].as<std::vector<float>>();
        for (size_t i = 0; i < bias.size() && i < cfg_bias.size(); ++i) {
            bias[i] = cfg_bias[i];
        }
    }

    const auto & force =
        source == "foot_force" ? env->robot->data.foot_force : env->robot->data.foot_force_est;

    std::vector<float> contacts(4, 0.0f);
    for (size_t i = 0; i < contacts.size(); ++i) {
        float corrected_force = force[i] - bias[i];
        if (use_abs) {
            corrected_force = std::abs(corrected_force);
        }
        contacts[i] = corrected_force > threshold ? 1.0f : 0.0f;
    }

    return contacts;
}

REGISTER_OBSERVATION(actor_observation_vector)
{
    std::vector<float> obs;

    auto append = [&](const std::vector<float>& v) {
        obs.insert(obs.end(), v.begin(), v.end());
    };

    append(isaaclab::mdp::base_linear_velocity(env, YAML::Node()));
    append(isaaclab::mdp::base_ang_vel(env, YAML::Node()));
    append(isaaclab::mdp::projected_gravity(env, YAML::Node()));
    append(isaaclab::mdp::joint_pos_rel(env, YAML::Node()));
    append(isaaclab::mdp::joint_vel_rel(env, YAML::Node()));
    append(isaaclab::mdp::last_action(env, YAML::Node()));
    append(isaaclab::mdp::foot_contacts_heuristic(env, YAML::Node()));
    append(isaaclab::mdp::twist_commands(env, YAML::Node()));
    append(isaaclab::mdp::arm_pose_command(env, YAML::Node()));

    for (auto & x : obs) {
        if (!std::isfinite(x)) {
            x = 0.0f;
        } else {
            x = std::clamp(x, -10.0f, 10.0f);
        }
    }

    return obs;
}

REGISTER_OBSERVATION(depth_student_actor_observation)
{
    constexpr size_t proprio_dim = 53;
    constexpr size_t zero_middle_dim = 170;
    constexpr size_t history_len = 10;
    constexpr size_t actor_obs_dim = proprio_dim + zero_middle_dim + proprio_dim * history_len;

    std::vector<float> proprio;
    proprio.reserve(proprio_dim);

    auto append = [&](const std::vector<float>& v, const size_t count, const float scale = 1.0f) {
        for (size_t i = 0; i < count; ++i) {
            proprio.push_back(i < v.size() ? v[i] * scale : 0.0f);
        }
    };
    auto reindex = [&](const std::vector<float>& v, const std::vector<int>& indices) {
        if (indices.empty()) {
            return v;
        }
        std::vector<float> out;
        out.reserve(indices.size());
        for (const int index : indices) {
            out.push_back(index >= 0 && index < static_cast<int>(v.size()) ? v[index] : 0.0f);
        }
        return out;
    };

    std::vector<int> joint_reindex;
    if (params && params["joint_reindex"] && !params["joint_reindex"].IsNull()) {
        joint_reindex = params["joint_reindex"].as<std::vector<int>>();
    }
    std::vector<int> foot_reindex;
    if (params && params["foot_reindex"] && !params["foot_reindex"].IsNull()) {
        foot_reindex = params["foot_reindex"].as<std::vector<int>>();
    }

    // Match astro_parkour.mdp.parkour_proprio:
    // 3 angular velocity, 2 gravity proxy, 3 yaw terms, 3 command terms,
    // 2 terrain flags, 12 joint positions, 12 joint velocities, 12 actions,
    // and 4 foot-contact flags.
    append(isaaclab::mdp::base_ang_vel(env, YAML::Node()), 3, 0.25f);
    append(isaaclab::mdp::projected_gravity(env, YAML::Node()), 2);

    const auto command_yaws = isaaclab::mdp::depth_student_command_yaws(env, params);
    if (params && params["yaw_debug"] && !params["yaw_debug"].IsNull()) {
        const auto yaw_debug = params["yaw_debug"];
        if (yaw_debug["enabled"].as<bool>(false)) {
            const long every_n_steps =
                std::max(1L, yaw_debug["every_n_steps"].as<long>(50));
            if (env->episode_length % every_n_steps == 0) {
                std::cout
                    << "[yaw_debug] step=" << env->episode_length
                    << " waypoint_delta_yaw=" << command_yaws.first
                    << " waypoint_delta_next_yaw=" << command_yaws.second
                    << std::endl;
            }
        }
    }
    proprio.push_back(0.0f); // zero_yaw, kept for training-layout compatibility.
    proprio.push_back(command_yaws.first);
    proprio.push_back(command_yaws.second);

    proprio.push_back(0.0f); // training fills the first two command slots with zeros.
    proprio.push_back(0.0f);

    const std::string command_name =
        params && params["command_name"] ? params["command_name"].as<std::string>() : "base_velocity";
    const auto command_cfg = env->cfg["commands"][command_name]["ranges"];
    const float forward_speed = isaaclab::mdp::depth_student_forward_speed(env, params, command_cfg);
    proprio.push_back(forward_speed * 2.0f);

    const bool flat_terrain =
        params && params["flat_terrain"] && !params["flat_terrain"].IsNull() &&
        params["flat_terrain"].as<bool>();
    proprio.push_back(flat_terrain ? 0.0f : 1.0f);
    proprio.push_back(flat_terrain ? 1.0f : 0.0f);

    append(reindex(isaaclab::mdp::joint_pos_rel(env, YAML::Node()), joint_reindex), 12);
    append(reindex(isaaclab::mdp::joint_vel_rel(env, YAML::Node()), joint_reindex), 12, 0.05f);
    append(isaaclab::mdp::last_action(env, YAML::Node()), 12);

    auto contacts = reindex(isaaclab::mdp::foot_contacts_from_force(env, params), foot_reindex);
    for (size_t i = 0; i < 4; ++i) {
        proprio.push_back((i < contacts.size() ? contacts[i] : 0.0f) - 0.5f);
    }

    proprio.resize(proprio_dim, 0.0f);
    for (auto& x : proprio) {
        if (!std::isfinite(x)) {
            x = 0.0f;
        }
    }

    auto history_proprio = proprio;
    history_proprio[6] = 0.0f;
    history_proprio[7] = 0.0f;

    static std::vector<float> history(proprio_dim * history_len, 0.0f);
    if (env->episode_length == 0) {
        for (size_t h = 0; h < history_len; ++h) {
            std::copy(history_proprio.begin(), history_proprio.end(), history.begin() + h * proprio_dim);
        }
    } else {
        std::copy(history.begin() + proprio_dim, history.end(), history.begin());
        std::copy(history_proprio.begin(), history_proprio.end(), history.end() - proprio_dim);
    }

    std::vector<float> obs;
    obs.reserve(actor_obs_dim);
    obs.insert(obs.end(), proprio.begin(), proprio.end());
    obs.insert(obs.end(), zero_middle_dim, 0.0f);
    obs.insert(obs.end(), history.begin(), history.end());
    return obs;
}

REGISTER_OBSERVATION(zero_depth_observation)
{
    const int height = params && params["height"] ? params["height"].as<int>() : 58;
    const int width = params && params["width"] ? params["width"].as<int>() : 87;
    return std::vector<float>(static_cast<size_t>(height * width), 0.0f);
}

REGISTER_OBSERVATION(depth_camera_observation)
{
    const auto preprocess_cfg = isaaclab::sensors::depth_preprocess_config_from_yaml(params);
    const auto debug_cfg = isaaclab::sensors::depth_debug_config_from_yaml(params);

    isaaclab::sensors::DepthFrame frame;
    const bool has_frame = isaaclab::sensors::get_depth_frame_from_provider(params, frame);
    if (!has_frame || !frame.valid) {
        auto fallback = isaaclab::sensors::fallback_depth_output(preprocess_cfg);
        const auto processed_stats = isaaclab::sensors::compute_depth_stats(fallback);
        const auto processed_counts = isaaclab::mdp::depth_zero_over20_counts(fallback);
        env->debug_telemetry.depth_valid = false;
        env->debug_telemetry.processed_depth_min = processed_stats.min;
        env->debug_telemetry.processed_depth_max = processed_stats.max;
        env->debug_telemetry.processed_depth_mean = processed_stats.mean;
        env->debug_telemetry.processed_depth_finite = static_cast<float>(processed_stats.finite_count);
        env->debug_telemetry.processed_depth_invalid = static_cast<float>(processed_stats.invalid_count);
        env->debug_telemetry.processed_depth_zero = static_cast<float>(processed_counts.first);
        env->debug_telemetry.processed_depth_over20 = static_cast<float>(processed_counts.second);
        isaaclab::sensors::maybe_debug_depth(frame, fallback, preprocess_cfg, debug_cfg);
        return fallback;
    }

    static double last_stamp = -1.0;
    static int step_count = 0;

    const double dt_from_last = last_stamp >= 0.0 ? frame.timestamp - last_stamp : 0.0;
    const bool repeated = frame.timestamp == last_stamp;

    if (++step_count % 20 == 0) {
        std::cout
            << "[depth_stamp] step=" << env->episode_length
            << " stamp=" << frame.timestamp
            << " dt_from_last=" << dt_from_last
            << " repeated=" << repeated
            << std::endl;
    }

    last_stamp = frame.timestamp;

    auto processed = isaaclab::sensors::preprocess_depth_frame(frame, preprocess_cfg);
    const auto raw_stats = isaaclab::sensors::compute_depth_stats(frame.data);
    const auto raw_counts = isaaclab::mdp::depth_zero_over20_counts(frame.data);
    const auto processed_stats = isaaclab::sensors::compute_depth_stats(processed);
    const auto processed_counts = isaaclab::mdp::depth_zero_over20_counts(processed);
    env->debug_telemetry.depth_valid = frame.valid;
    env->debug_telemetry.depth_stamp = frame.timestamp;
    env->debug_telemetry.depth_dt_from_last = dt_from_last;
    env->debug_telemetry.depth_repeated = repeated;
    env->debug_telemetry.raw_depth_min = raw_stats.min;
    env->debug_telemetry.raw_depth_max = raw_stats.max;
    env->debug_telemetry.raw_depth_mean = raw_stats.mean;
    env->debug_telemetry.raw_depth_finite = static_cast<float>(raw_stats.finite_count);
    env->debug_telemetry.raw_depth_invalid = static_cast<float>(raw_stats.invalid_count);
    env->debug_telemetry.raw_depth_zero = static_cast<float>(raw_counts.first);
    env->debug_telemetry.raw_depth_over20 = static_cast<float>(raw_counts.second);
    env->debug_telemetry.processed_depth_min = processed_stats.min;
    env->debug_telemetry.processed_depth_max = processed_stats.max;
    env->debug_telemetry.processed_depth_mean = processed_stats.mean;
    env->debug_telemetry.processed_depth_finite = static_cast<float>(processed_stats.finite_count);
    env->debug_telemetry.processed_depth_invalid = static_cast<float>(processed_stats.invalid_count);
    env->debug_telemetry.processed_depth_zero = static_cast<float>(processed_counts.first);
    env->debug_telemetry.processed_depth_over20 = static_cast<float>(processed_counts.second);
    isaaclab::sensors::maybe_debug_depth(frame, processed, preprocess_cfg, debug_cfg);
    return processed;
}

REGISTER_OBSERVATION(gait_phase)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    auto cmd = isaaclab::mdp::twist_commands(env, params);
    float cmd_norm = std::sqrt(
        cmd[0] * cmd[0] +
        cmd[1] * cmd[1] +
        cmd[2] * cmd[2]
    );

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);

    if (cmd_norm < 0.1f)
    {
        obs[0] = 0.0f;
        obs[1] = 0.0f;
    }

    return obs;
}

}
}
