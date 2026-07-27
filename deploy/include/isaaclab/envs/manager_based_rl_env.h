// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "param.h"
#include "isaaclab/manager/observation_manager.h"
#include "isaaclab/manager/action_manager.h"
#include "isaaclab/assets/articulation/articulation.h"
#include "isaaclab/algorithms/algorithms.h"
#include <iostream>
#include "isaaclab/utils/utils.h"

namespace isaaclab
{

class ObservationManager;
class ActionManager;

class ManagerBasedRLEnv
{
public:
    struct DebugTelemetry
    {
        float origin_x = 0.0f;
        float origin_y = 0.0f;
        float progress_x = 0.0f;
        float robot_yaw = 0.0f;
        int waypoint_idx = -1;
        int reach_timer = -1;
        float waypoint_state_x = 0.0f;
        float waypoint_state_y = 0.0f;
        float target_x = 0.0f;
        float target_y = 0.0f;
        float next_x = 0.0f;
        float next_y = 0.0f;
        float dist_to_target = 0.0f;
        float dist_to_next = 0.0f;
        float waypoint_delta_yaw = 0.0f;
        float waypoint_delta_next_yaw = 0.0f;
        double depth_stamp = 0.0;
        double depth_dt_from_last = 0.0;
        float raw_depth_min = 0.0f;
        float raw_depth_max = 0.0f;
        float raw_depth_mean = 0.0f;
        float raw_depth_finite = 0.0f;
        float raw_depth_invalid = 0.0f;
        float raw_depth_zero = 0.0f;
        float raw_depth_over20 = 0.0f;
        float processed_depth_min = 0.0f;
        float processed_depth_max = 0.0f;
        float processed_depth_mean = 0.0f;
        float processed_depth_finite = 0.0f;
        float processed_depth_invalid = 0.0f;
        float processed_depth_zero = 0.0f;
        float processed_depth_over20 = 0.0f;
        bool depth_repeated = false;
        bool depth_valid = false;
    };

    // Constructor
    ManagerBasedRLEnv(YAML::Node cfg, std::shared_ptr<Articulation> robot_)
    :cfg(cfg), robot(std::move(robot_))
    {
        // Parse configuration
        this->step_dt = cfg["step_dt"].as<float>();
        robot->data.joint_ids_map = cfg["joint_ids_map"].as<std::vector<float>>();
        robot->data.joint_pos.resize(robot->data.joint_ids_map.size());
        robot->data.joint_vel.resize(robot->data.joint_ids_map.size());

        { // default joint positions
            auto default_joint_pos = cfg["default_joint_pos"].as<std::vector<float>>();
            robot->data.default_joint_pos = Eigen::VectorXf::Map(default_joint_pos.data(), default_joint_pos.size());
        }
        { // joint stiffness and damping
            robot->data.joint_stiffness = cfg["stiffness"].as<std::vector<float>>();
            robot->data.joint_damping = cfg["damping"].as<std::vector<float>>();
        }

        robot->update();

        // load managers
        action_manager = std::make_unique<ActionManager>(cfg["actions"], this);
        observation_manager = std::make_unique<ObservationManager>(cfg["observations"], this);

        if (cfg["debug_logger"])
        {
            const auto logger_cfg = cfg["debug_logger"];
            debug_logger_enabled_ = logger_cfg["enabled"].as<bool>(false);
            debug_print_enabled_ = logger_cfg["print_to_console"].as<bool>(false);
            debug_log_csv_enabled_ = logger_cfg["write_csv"].as<bool>(false);
            debug_every_n_steps_ = std::max(1, logger_cfg["every_n_steps"].as<int>(1));
            debug_csv_path_ = logger_cfg["csv_path"].as<std::string>("debug_obs_action.csv");
        }

        if (debug_log_csv_enabled_)
        {
            init_debug_csv();
        }
    }

    void reset()
    {
        global_phase = 0;
        episode_length = 0;
        robot->update();
        action_manager->reset();
        observation_manager->reset();
        if (alg)
        {
            alg->reset();
        }
    }

    void step()
    {
        episode_length += 1;
        robot->update();
        auto obs = observation_manager->compute();
        auto action = alg->act(obs);
        action_manager->process_action(action);
        auto processed_action = action_manager->processed_actions();

        maybe_log_debug(obs, action, processed_action);
    }

    float step_dt;
    
    YAML::Node cfg;

    std::unique_ptr<ObservationManager> observation_manager;
    std::unique_ptr<ActionManager> action_manager;
    std::shared_ptr<Articulation> robot;
    std::unique_ptr<Algorithms> alg;
    long episode_length = 0;
    float global_phase = 0.0f;
    DebugTelemetry debug_telemetry;

private:
    void init_debug_csv()
    {
        std::filesystem::path csv_path = debug_csv_path_;
        if (csv_path.is_relative())
        {
            csv_path = param::proj_dir / csv_path;
        }
        std::filesystem::create_directories(csv_path.parent_path());
        debug_csv_stream_.open(csv_path, std::ios::out | std::ios::trunc);
        if (!debug_csv_stream_.is_open())
        {
            throw std::runtime_error("Failed to open debug csv file: " + csv_path.string());
        }
        debug_csv_path_ = csv_path.string();
        spdlog::info("Debug observation/action CSV: {}", debug_csv_path_);
    }

    std::vector<std::string> sorted_obs_group_names(
        const std::unordered_map<std::string, std::vector<float>>& obs) const
    {
        std::vector<std::string> group_names;
        group_names.reserve(obs.size());
        for (const auto& item : obs)
        {
            group_names.push_back(item.first);
        }
        std::sort(group_names.begin(), group_names.end());
        return group_names;
    }

    void write_debug_csv_header(const std::unordered_map<std::string, std::vector<float>>& obs,
                                const std::vector<float>& raw_action,
                                const std::vector<float>& processed_action)
    {
        if (!debug_csv_stream_.is_open() || debug_csv_header_written_)
        {
            return;
        }

        debug_csv_stream_ << "step";
        for (const auto& group_name : sorted_obs_group_names(obs))
        {
            const auto& values = obs.at(group_name);
            for (int i = 0; i < values.size(); ++i)
            {
                debug_csv_stream_ << ",obs_" << group_name << "_" << i;
            }
        }
        for (int i = 0; i < raw_action.size(); ++i)
        {
            debug_csv_stream_ << ",raw_action_" << i;
        }
        for (int i = 0; i < processed_action.size(); ++i)
        {
            debug_csv_stream_ << ",processed_action_" << i;
        }
        write_debug_telemetry_header();
        debug_csv_stream_ << "\n";
        debug_csv_header_written_ = true;
    }

    void write_debug_telemetry_header()
    {
        debug_csv_stream_
            << ",debug_sim_time"
            << ",debug_step_dt"
            << ",debug_origin_x"
            << ",debug_origin_y"
            << ",debug_progress_x"
            << ",debug_root_pos_x"
            << ",debug_root_pos_y"
            << ",debug_root_pos_z"
            << ",debug_root_lin_vel_b_x"
            << ",debug_root_lin_vel_b_y"
            << ",debug_root_lin_vel_b_z"
            << ",debug_root_ang_vel_b_x"
            << ",debug_root_ang_vel_b_y"
            << ",debug_root_ang_vel_b_z"
            << ",debug_projected_gravity_x"
            << ",debug_projected_gravity_y"
            << ",debug_projected_gravity_z"
            << ",debug_robot_yaw"
            << ",debug_waypoint_idx"
            << ",debug_reach_timer"
            << ",debug_waypoint_state_x"
            << ",debug_waypoint_state_y"
            << ",debug_target_x"
            << ",debug_target_y"
            << ",debug_next_x"
            << ",debug_next_y"
            << ",debug_dist_to_target"
            << ",debug_dist_to_next"
            << ",debug_waypoint_delta_yaw"
            << ",debug_waypoint_delta_next_yaw"
            << ",debug_depth_stamp"
            << ",debug_depth_dt_from_last"
            << ",debug_depth_repeated"
            << ",debug_depth_valid"
            << ",debug_raw_depth_min"
            << ",debug_raw_depth_max"
            << ",debug_raw_depth_mean"
            << ",debug_raw_depth_finite"
            << ",debug_raw_depth_invalid"
            << ",debug_raw_depth_zero"
            << ",debug_raw_depth_over20"
            << ",debug_processed_depth_min"
            << ",debug_processed_depth_max"
            << ",debug_processed_depth_mean"
            << ",debug_processed_depth_finite"
            << ",debug_processed_depth_invalid"
            << ",debug_processed_depth_zero"
            << ",debug_processed_depth_over20";
    }

    static std::string summarize_vector(const std::vector<float>& values)
    {
        std::ostringstream oss;
        oss << "[";
        for (int i = 0; i < values.size(); ++i)
        {
            if (i > 0)
            {
                oss << ", ";
            }
            oss << std::fixed << std::setprecision(4) << values[i];
        }
        oss << "]";
        return oss.str();
    }

    void maybe_print_debug(const std::unordered_map<std::string, std::vector<float>>& obs,
                           const std::vector<float>& raw_action,
                           const std::vector<float>& processed_action)
    {
        if (!debug_print_enabled_)
        {
            return;
        }

        std::ostringstream oss;
        oss << "step=" << episode_length;
        for (const auto& group_name : sorted_obs_group_names(obs))
        {
            oss << " " << group_name << "=" << summarize_vector(obs.at(group_name));
        }
        oss << " raw_action=" << summarize_vector(raw_action);
        oss << " processed_action=" << summarize_vector(processed_action);
        spdlog::info("Deploy debug {}", oss.str());
    }

    void maybe_write_debug_csv(const std::unordered_map<std::string, std::vector<float>>& obs,
                               const std::vector<float>& raw_action,
                               const std::vector<float>& processed_action)
    {
        if (!debug_log_csv_enabled_ || !debug_csv_stream_.is_open())
        {
            return;
        }

        write_debug_csv_header(obs, raw_action, processed_action);

        debug_csv_stream_ << episode_length;
        for (const auto& group_name : sorted_obs_group_names(obs))
        {
            const auto& values = obs.at(group_name);
            for (const auto value : values)
            {
                debug_csv_stream_ << "," << std::fixed << std::setprecision(9) << value;
            }
        }
        for (const auto value : raw_action)
        {
            debug_csv_stream_ << "," << std::fixed << std::setprecision(9) << value;
        }
        for (const auto value : processed_action)
        {
            debug_csv_stream_ << "," << std::fixed << std::setprecision(9) << value;
        }
        write_debug_telemetry_row();
        debug_csv_stream_ << "\n";
        debug_csv_stream_.flush();
    }

    void write_debug_telemetry_row()
    {
        const auto& t = debug_telemetry;
        debug_csv_stream_
            << "," << std::fixed << std::setprecision(9) << episode_length * step_dt
            << "," << step_dt
            << "," << t.origin_x
            << "," << t.origin_y
            << "," << t.progress_x
            << "," << robot->data.root_pos_w.x()
            << "," << robot->data.root_pos_w.y()
            << "," << robot->data.root_pos_w.z()
            << "," << robot->data.root_lin_vel_b.x()
            << "," << robot->data.root_lin_vel_b.y()
            << "," << robot->data.root_lin_vel_b.z()
            << "," << robot->data.root_ang_vel_b.x()
            << "," << robot->data.root_ang_vel_b.y()
            << "," << robot->data.root_ang_vel_b.z()
            << "," << robot->data.projected_gravity_b.x()
            << "," << robot->data.projected_gravity_b.y()
            << "," << robot->data.projected_gravity_b.z()
            << "," << t.robot_yaw
            << "," << t.waypoint_idx
            << "," << t.reach_timer
            << "," << t.waypoint_state_x
            << "," << t.waypoint_state_y
            << "," << t.target_x
            << "," << t.target_y
            << "," << t.next_x
            << "," << t.next_y
            << "," << t.dist_to_target
            << "," << t.dist_to_next
            << "," << t.waypoint_delta_yaw
            << "," << t.waypoint_delta_next_yaw
            << "," << t.depth_stamp
            << "," << t.depth_dt_from_last
            << "," << (t.depth_repeated ? 1.0f : 0.0f)
            << "," << (t.depth_valid ? 1.0f : 0.0f)
            << "," << t.raw_depth_min
            << "," << t.raw_depth_max
            << "," << t.raw_depth_mean
            << "," << t.raw_depth_finite
            << "," << t.raw_depth_invalid
            << "," << t.raw_depth_zero
            << "," << t.raw_depth_over20
            << "," << t.processed_depth_min
            << "," << t.processed_depth_max
            << "," << t.processed_depth_mean
            << "," << t.processed_depth_finite
            << "," << t.processed_depth_invalid
            << "," << t.processed_depth_zero
            << "," << t.processed_depth_over20;
    }

    void maybe_log_debug(const std::unordered_map<std::string, std::vector<float>>& obs,
                         const std::vector<float>& raw_action,
                         const std::vector<float>& processed_action)
    {
        if (!debug_logger_enabled_)
        {
            return;
        }
        if (episode_length % debug_every_n_steps_ != 0)
        {
            return;
        }

        maybe_print_debug(obs, raw_action, processed_action);
        maybe_write_debug_csv(obs, raw_action, processed_action);
    }

    bool debug_logger_enabled_ = false;
    bool debug_print_enabled_ = false;
    bool debug_log_csv_enabled_ = false;
    bool debug_csv_header_written_ = false;
    int debug_every_n_steps_ = 1;
    std::string debug_csv_path_ = "debug_obs_action.csv";
    std::ofstream debug_csv_stream_;
};

};
