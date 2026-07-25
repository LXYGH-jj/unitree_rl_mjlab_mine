#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "unitree/common/time/time_tool.hpp"
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

class ArmCommandManager
{
public:
    enum class Mode
    {
        Off,
        MoveToStart,
        TrackTrajectory,
        ReturnToStart,
        MoveToZero
    };

    static ArmCommandManager& instance()
    {
        static ArmCommandManager manager;
        return manager;
    }

    bool joint_trajectory_enabled(const YAML::Node& cfg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto joint_cfg = cfg["joint_trajectory"];
        return joint_cfg && joint_cfg["enabled"] && joint_cfg["enabled"].as<bool>();
    }

    std::vector<float> pose_command(const YAML::Node& cfg, unitree::common::UnitreeJoystick* joystick)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = current_time();

        auto start_pose = read_vec(cfg["start_pose"], read_vec(cfg["default_command"],
            {0.5f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f}, 7), 7);

        update_toggle(joystick, now, cfg, start_pose);
        advance(cfg, now);

        if (mode_ == Mode::MoveToStart) {
            const float duration = cfg["move_to_start_duration"].as<float>(2.0f);
            return lerp_vec(pose_transition_start_cmd_, start_pose, duration, now - state_start_time_);
        }

        if (mode_ == Mode::ReturnToStart) {
            const float duration = cfg["return_to_start_duration"].as<float>(2.0f);
            return lerp_vec(pose_transition_start_cmd_, start_pose, duration, now - state_start_time_);
        }

        if (mode_ != Mode::TrackTrajectory) {
            return start_pose;
        }

        const auto trajectory = cfg["trajectory"];
        if (!trajectory || !trajectory["enabled"] || !trajectory["enabled"].as<bool>()) {
            return start_pose;
        }

        return sample_pose_trajectory_unlocked(trajectory, start_pose, now - state_start_time_);
    }

    std::vector<float> sample_pose_trajectory_unlocked(
        const YAML::Node& trajectory,
        const std::vector<float>& start_pose,
        double elapsed) const
    {
        const float period = trajectory["period"].as<float>(7.0f);
        const float dwell_time = trajectory["dwell_time"].as<float>(0.0f);
        const float phase = trajectory["phase"].as<float>(0.0f);
        const auto center = read_vec(trajectory["center"], start_pose, 7);
        const auto amplitude = read_vec(trajectory["amplitude"], {0.0f, 0.22f, 0.04f}, 3);

        std::vector<float> cmd = center;
        const float t = static_cast<float>(elapsed) + phase;

        if (dwell_time > 1.0e-3f && period > 2.0f * dwell_time + 1.0e-3f) {
            float cycle = std::fmod(t, period);
            if (cycle < 0.0f) {
                cycle += period;
            }

            const float move_time = 0.5f * (period - 2.0f * dwell_time);
            float sweep = 1.0f;
            if (cycle < dwell_time) {
                sweep = 1.0f;
            } else if (cycle < dwell_time + move_time) {
                const float u = (cycle - dwell_time) / move_time;
                sweep = std::cos(static_cast<float>(M_PI) * u);
            } else if (cycle < 2.0f * dwell_time + move_time) {
                sweep = -1.0f;
            } else {
                const float u = (cycle - 2.0f * dwell_time - move_time) / move_time;
                sweep = -std::cos(static_cast<float>(M_PI) * u);
            }

            cmd[0] = center[0] + amplitude[0] * sweep;
            cmd[1] = center[1] + amplitude[1] * sweep;
            cmd[2] = center[2] + amplitude[2] * (1.0f - sweep * sweep);
        } else {
            const float omega = period > 1.0e-3f ? (2.0f * static_cast<float>(M_PI) / period) : 0.0f;
            const float s = std::sin(omega * t);
            const float c = std::cos(omega * t);
            cmd[0] = center[0] + amplitude[0] * c;
            cmd[1] = center[1] + amplitude[1] * s;
            cmd[2] = center[2] + amplitude[2] * s;
        }

        const float ramp_in_duration = trajectory["ramp_in_duration"].as<float>(1.5f);
        return lerp_vec(start_pose, cmd, ramp_in_duration, elapsed);
    }

    std::vector<float> joint_command_q(const YAML::Node& cfg, const std::vector<float>& current_q)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = current_time();
        advance(cfg, now);

        const size_t n = current_q.size();
        const auto joint_cfg = cfg["joint_trajectory"];
        auto zero_q = read_vec(cfg["zero_q"], std::vector<float>(n, 0.0f), n);

        if (!joint_cfg || !joint_cfg["enabled"] || !joint_cfg["enabled"].as<bool>()) {
            return zero_q;
        }

        capture_transition_start_unlocked(current_q);

        auto start_q = read_vec(joint_cfg["start_q"], zero_q, n);
        if (mode_ == Mode::Off) {
            return zero_q;
        }

        const double elapsed = now - state_start_time_;
        if (mode_ == Mode::MoveToStart) {
            const float duration = cfg["move_to_start_duration"].as<float>(2.0f);
            return lerp_vec(transition_start_q_, start_q, duration, elapsed);
        }

        if (mode_ == Mode::TrackTrajectory) {
            return sample_joint_trajectory_unlocked(joint_cfg, start_q);
        }

        if (mode_ == Mode::ReturnToStart) {
            const float duration = cfg["return_to_start_duration"].as<float>(2.0f);
            return lerp_vec(transition_start_q_, start_q, duration, elapsed);
        }

        const float duration = cfg["move_to_zero_duration"].as<float>(2.0f);
        return lerp_vec(transition_start_q_, zero_q, duration, elapsed);
    }

    bool should_override_arm_to_zero(const YAML::Node& cfg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const double now = current_time();
        advance(cfg, now);
        return mode_ == Mode::MoveToZero || mode_ == Mode::Off;
    }

    std::vector<int> zero_indices(const YAML::Node& cfg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (cfg["zero_indices"] && !cfg["zero_indices"].IsNull()) {
            return cfg["zero_indices"].as<std::vector<int>>();
        }
        return {12, 13, 14, 15, 16, 17};
    }

    std::vector<float> zero_q(const YAML::Node& cfg, size_t n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto q = read_vec(cfg["zero_q"], std::vector<float>(n, 0.0f), n);
        q.resize(n, 0.0f);
        return q;
    }

    std::vector<float> target_q(const YAML::Node& cfg, size_t n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ == Mode::Off && off_hold_valid_) {
            auto q = off_hold_q_;
            q.resize(n, 0.0f);
            return q;
        }
        auto q = read_vec(cfg["zero_q"], std::vector<float>(n, 0.0f), n);
        q.resize(n, 0.0f);
        return q;
    }

    float zero_ratio(const YAML::Node& cfg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ == Mode::Off) {
            return 1.0f;
        }
        const float duration = cfg["move_to_zero_duration"].as<float>(2.0f);
        if (duration <= 1.0e-3f) {
            return 1.0f;
        }
        return std::clamp(static_cast<float>(current_time() - state_start_time_) / duration, 0.0f, 1.0f);
    }

    void capture_policy_takeover_start(const std::vector<float>& q)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != Mode::MoveToStart || policy_takeover_start_valid_) {
            return;
        }
        policy_takeover_start_q_ = q;
        policy_takeover_start_valid_ = true;
    }

    std::vector<float> blend_policy_takeover_q(
        const YAML::Node& cfg,
        const std::vector<float>& policy_q,
        size_t n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != Mode::MoveToStart) {
            auto q = policy_q;
            q.resize(n, 0.0f);
            return q;
        }

        auto from = policy_takeover_start_q_;
        from.resize(n, 0.0f);
        auto to = policy_q;
        to.resize(n, 0.0f);
        const float duration = cfg["policy_takeover_duration"].as<float>(
            cfg["move_to_start_duration"].as<float>(2.0f));
        return lerp_vec(from, to, duration, current_time() - state_start_time_);
    }

    void capture_zero_start(const std::vector<float>& q)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != Mode::MoveToZero || zero_start_valid_) {
            return;
        }
        zero_start_q_ = q;
        zero_start_valid_ = true;
    }

    void capture_off_hold(const std::vector<float>& q)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ != Mode::Off || off_hold_valid_) {
            return;
        }
        off_hold_q_ = q;
        off_hold_valid_ = true;
    }

    std::vector<float> zero_start_q(size_t n)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto q = zero_start_q_;
        q.resize(n, 0.0f);
        return q;
    }

    std::string mode_name()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        switch (mode_) {
            case Mode::Off: return "Off";
            case Mode::MoveToStart: return "MoveToStart";
            case Mode::TrackTrajectory: return "TrackTrajectory";
            case Mode::ReturnToStart: return "ReturnToStart";
            case Mode::MoveToZero: return "MoveToZero";
        }
        return "Unknown";
    }

private:
    static double current_time()
    {
        return static_cast<double>(unitree::common::GetCurrentTimeMillisecond()) * 1.0e-3;
    }

    static std::vector<float> read_vec(const YAML::Node& node, std::vector<float> fallback, size_t expected)
    {
        if (node && !node.IsNull()) {
            auto value = node.as<std::vector<float>>();
            if (value.size() == expected) {
                return value;
            }
        }
        fallback.resize(expected, 0.0f);
        return fallback;
    }

    void transition(Mode mode, double now)
    {
        mode_ = mode;
        state_start_time_ = now;
        zero_start_valid_ = false;
        zero_start_q_.clear();
        off_hold_valid_ = false;
        off_hold_q_.clear();
        transition_start_valid_ = false;
        transition_start_q_.clear();
        policy_takeover_start_valid_ = false;
        policy_takeover_start_q_.clear();
    }

    static std::vector<float> lerp_vec(
        const std::vector<float>& from,
        const std::vector<float>& to,
        float duration,
        double elapsed)
    {
        if (duration <= 1.0e-3f) {
            return to;
        }
        const float alpha = std::clamp(static_cast<float>(elapsed / duration), 0.0f, 1.0f);
        std::vector<float> out(to.size(), 0.0f);
        for (size_t i = 0; i < to.size(); ++i) {
            const float a = i < from.size() ? from[i] : 0.0f;
            out[i] = a + alpha * (to[i] - a);
        }
        return out;
    }

    void capture_transition_start_unlocked(const std::vector<float>& q)
    {
        if (transition_start_valid_) {
            return;
        }
        transition_start_q_ = q;
        transition_start_valid_ = true;
    }

    std::vector<float> sample_joint_trajectory_unlocked(
        const YAML::Node& joint_cfg,
        const std::vector<float>& start_q) const
    {
        struct Waypoint {
            float time_from_start = 0.0f;
            std::vector<float> q;
        };

        std::vector<Waypoint> waypoints;
        if (joint_cfg["waypoints"] && joint_cfg["waypoints"].IsSequence()) {
            for (const auto& node : joint_cfg["waypoints"]) {
                if (!node["time_from_start"] || !node["positions"]) {
                    continue;
                }
                Waypoint wp;
                wp.time_from_start = node["time_from_start"].as<float>();
                wp.q = read_vec(node["positions"], start_q, start_q.size());
                waypoints.push_back(std::move(wp));
            }
        }

        if (waypoints.empty()) {
            waypoints.push_back({2.0f, start_q});
            waypoints.push_back({4.0f, {0.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f}});
            waypoints.push_back({6.0f, {static_cast<float>(M_PI) / 4.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f}});
            waypoints.push_back({8.0f, start_q});
        }

        const float duration = std::max(waypoints.back().time_from_start, 1.0e-3f);
        const bool loop = joint_cfg["loop"] ? joint_cfg["loop"].as<bool>() : true;
        float t = static_cast<float>(current_time() - state_start_time_);
        if (loop) {
            t = std::fmod(t, duration);
            if (t < 0.0f) {
                t += duration;
            }
        } else {
            t = std::clamp(t, 0.0f, duration);
        }

        if (t <= waypoints.front().time_from_start) {
            return waypoints.front().q;
        }

        for (size_t i = 1; i < waypoints.size(); ++i) {
            if (t <= waypoints[i].time_from_start) {
                const auto& a = waypoints[i - 1];
                const auto& b = waypoints[i];
                const float span = std::max(b.time_from_start - a.time_from_start, 1.0e-3f);
                const float alpha = std::clamp((t - a.time_from_start) / span, 0.0f, 1.0f);
                return lerp_vec(a.q, b.q, 1.0f, alpha);
            }
        }

        return waypoints.back().q;
    }

    void update_toggle(
        unitree::common::UnitreeJoystick* joystick,
        double now,
        const YAML::Node& cfg,
        const std::vector<float>& start_pose)
    {
        if (!joystick) {
            return;
        }

        if (!joystick->Y.pressed) {
            toggle_latched_ = false;
            return;
        }

        if (!joystick->Y.on_pressed || toggle_latched_) {
            return;
        }

        toggle_latched_ = true;
        if (mode_ == Mode::Off || mode_ == Mode::MoveToZero) {
            transition(Mode::MoveToStart, now);
            pose_transition_start_cmd_ = start_pose;
        } else {
            std::vector<float> current_pose_cmd = start_pose;
            const auto trajectory = cfg["trajectory"];
            if (trajectory && trajectory["enabled"] && trajectory["enabled"].as<bool>()) {
                current_pose_cmd =
                    sample_pose_trajectory_unlocked(trajectory, start_pose, now - state_start_time_);
            }
            transition(Mode::ReturnToStart, now);
            pose_transition_start_cmd_ = current_pose_cmd;
        }
    }

    void advance(const YAML::Node& cfg, double now)
    {
        const float move_to_start = cfg["move_to_start_duration"].as<float>(2.0f);
        const float start_hold = cfg["start_hold_duration"].as<float>(0.0f);
        const float return_to_start = cfg["return_to_start_duration"].as<float>(2.0f);
        const float return_hold = cfg["return_hold_duration"].as<float>(0.0f);
        const float move_to_zero = cfg["move_to_zero_duration"].as<float>(2.0f);
        const double elapsed = now - state_start_time_;

        if (mode_ == Mode::MoveToStart && elapsed >= move_to_start + start_hold) {
            transition(Mode::TrackTrajectory, now);
        } else if (mode_ == Mode::ReturnToStart && elapsed >= return_to_start + return_hold) {
            transition(Mode::MoveToZero, now);
        } else if (mode_ == Mode::MoveToZero && elapsed >= move_to_zero) {
            transition(Mode::Off, now);
        }
    }

    std::mutex mutex_;
    Mode mode_ = Mode::Off;
    double state_start_time_ = 0.0;
    bool toggle_latched_ = false;
    bool zero_start_valid_ = false;
    bool off_hold_valid_ = false;
    bool transition_start_valid_ = false;
    bool policy_takeover_start_valid_ = false;
    std::vector<float> zero_start_q_;
    std::vector<float> off_hold_q_;
    std::vector<float> transition_start_q_;
    std::vector<float> pose_transition_start_cmd_;
    std::vector<float> policy_takeover_start_q_;
};
