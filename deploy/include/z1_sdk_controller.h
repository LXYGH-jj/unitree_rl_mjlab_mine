#pragma once

#include <atomic>
#include <chrono>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <unitree_arm_sdk/control/unitreeArm.h>

class Z1SdkController
{
public:
    explicit Z1SdkController(const YAML::Node& cfg);
    ~Z1SdkController();

    bool enabled() const { return enabled_; }
    bool has_state() const;

    std::vector<float> current_q(const std::vector<float>& fallback) const;
    std::vector<float> current_dq(size_t n) const;
    std::vector<float> current_tau(size_t n) const;
    std::vector<float> command_q(size_t n) const;

    void update(
        const std::vector<float>& target_q,
        const std::vector<float>& target_dq = {},
        const std::vector<float>& target_tau = {});
    void hold_current();

private:
    void control_loop();

    bool enabled_ = false;
    bool with_gripper_ = false;
    bool passive_on_shutdown_ = true;
    bool back_to_start_on_shutdown_ = false;
    double control_dt_sec_ = 0.002;
    bool launch_controller_process_ = true;
    double controller_startup_delay_sec_ = 1.0;
    double controller_shutdown_delay_sec_ = 4.0;
    std::string controller_binary_path_;
    int controller_pid_ = -1;

    std::unique_ptr<UNITREE_ARM::unitreeArm> arm_;
    std::thread control_thread_;
    std::atomic<bool> running_ {false};

    mutable std::mutex mutex_;
    std::vector<float> current_q_;
    std::vector<float> current_dq_;
    std::vector<float> current_tau_;
    std::vector<float> command_q_;
    std::vector<float> command_dq_;
    std::vector<float> command_tau_;
    bool state_valid_ = false;
    // This timestamp is an observation time, NOT proof of receipt of a fresh UDP packet.
    // Independent packet freshness still requires verification prior to hardware enablement.
    std::chrono::steady_clock::time_point last_state_observed_{};
    bool sdk_fault_latched_ = false;

    void start_controller_process();
    void stop_controller_process();
};
