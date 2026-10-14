#include "z1_sdk_controller.h"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <cmath>
#include <stdexcept>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <spdlog/spdlog.h>

namespace
{
constexpr size_t kArmJointCount = 6;

#ifndef B2Z1_DEPLOY_ROOT
#define B2Z1_DEPLOY_ROOT "."
#endif

std::vector<float> resize_command(const std::vector<float>& input, size_t n)
{
    auto output = input;
    output.resize(n, 0.0f);
    return output;
}

template <typename Container>
std::vector<float> read_arm_array(const Container& source, size_t n)
{
    std::vector<float> values(n, 0.0f);
    for (size_t i = 0; i < n && i < source.size(); ++i) {
        values[i] = static_cast<float>(source[i]);
    }
    return values;
}

Vec6 to_vec6(const std::vector<float>& input)
{
    Vec6 output = Vec6::Zero();
    for (size_t i = 0; i < std::min(kArmJointCount, input.size()); ++i) {
        output[static_cast<Eigen::Index>(i)] = static_cast<double>(input[i]);
    }
    return output;
}

std::filesystem::path deploy_root_path()
{
    return std::filesystem::path(B2Z1_DEPLOY_ROOT);
}

std::filesystem::path resolve_deploy_path(const std::string& path_string)
{
    std::filesystem::path path(path_string);
    if (path.is_absolute()) {
        return path;
    }
    return deploy_root_path() / path;
}
}

Z1SdkController::Z1SdkController(const YAML::Node& cfg)
{
    if (!cfg || !cfg["enabled"] || !cfg["enabled"].as<bool>()) {
        return;
    }

    // Mandatory fail-closed opt-in. Legacy z1_sdk.enabled=true NEVER implies
    // authorization to energize the Z1 arm or start z1_ctrl.
    const bool hardware_ack = cfg["hardware_io_authorized"].as<bool>(false);
    const char* env_ack = std::getenv("B2Z1_Z1_HARDWARE_ACK");
    if (!hardware_ack || !env_ack ||
        std::string(env_ack) != "Z1_BENCH_ELECTRICAL_CHECK_COMPLETE") {
        throw std::runtime_error(
            "Z1 hardware I/O blocked: requires explicit electrical inspection and "
            "two-factor bench authorization. Do not set it for offline tests.");
    }
    with_gripper_ = cfg["with_gripper"].as<bool>(with_gripper_);
    passive_on_shutdown_ = cfg["passive_on_shutdown"].as<bool>(passive_on_shutdown_);
    back_to_start_on_shutdown_ = cfg["back_to_start_on_shutdown"].as<bool>(back_to_start_on_shutdown_);
    if (back_to_start_on_shutdown_)
        throw std::runtime_error("Z1 auto backToStart on shutdown forbidden");
    launch_controller_process_ = cfg["launch_controller_process"].as<bool>(launch_controller_process_);
    controller_startup_delay_sec_ =
        cfg["controller_startup_delay_sec"].as<double>(controller_startup_delay_sec_);
    controller_shutdown_delay_sec_ =
        cfg["controller_shutdown_delay_sec"].as<double>(controller_shutdown_delay_sec_);
    controller_binary_path_ = cfg["controller_binary_path"].as<std::string>(
        "thirdparty/z1_ctrl/z1_ctrl");

    const double control_hz = cfg["control_hz"].as<double>(500.0);
    control_dt_sec_ = control_hz > 1.0 ? 1.0 / control_hz : control_dt_sec_;

    current_q_.assign(kArmJointCount, 0.0f);
    current_dq_.assign(kArmJointCount, 0.0f);
    current_tau_.assign(kArmJointCount, 0.0f);
    command_q_.assign(kArmJointCount, 0.0f);
    command_dq_.assign(kArmJointCount, 0.0f);
    command_tau_.assign(kArmJointCount, 0.0f);

    start_controller_process();

    try {
        arm_ = std::make_unique<UNITREE_ARM::unitreeArm>(with_gripper_);
        // The vendor background thread is used ONLY for initial FSM transitions.
        // It MUST be stopped before our 500 Hz control_loop begins sendRecv().
        arm_->sendRecvThread->start();
        if (!arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE))
            throw std::runtime_error("Z1 PASSIVE transition failed; LOWCMD prohibited");

        if (!arm_->lowstate || !arm_->_ctrlComp || !arm_->_ctrlComp->udp ||
            arm_->_ctrlComp->udp->isDisConnect ||
            arm_->lowstate->q.size() < kArmJointCount ||
            arm_->lowstate->dq.size() < kArmJointCount ||
            arm_->lowstate->tau.size() < kArmJointCount ||
            arm_->lowstate->errorstate.size() < kArmJointCount ||
            arm_->lowstate->isMotorConnected.size() < kArmJointCount)
            throw std::runtime_error("Z1 initial feedback missing/incomplete/disconnected");

        for (size_t i=0; i<kArmJointCount; ++i) {
            if (!std::isfinite(arm_->lowstate->q[i]) ||
                !std::isfinite(arm_->lowstate->dq[i]) ||
                !std::isfinite(arm_->lowstate->tau[i]) ||
                (arm_->lowstate->errorstate[i] & static_cast<uint8_t>(~0x40u)) != 0 ||
                arm_->lowstate->isMotorConnected[i] != 0)
                throw std::runtime_error("Z1 invalid initial joint or motor error");
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            current_q_ = read_arm_array(arm_->lowstate->q, kArmJointCount);
            current_dq_ = read_arm_array(arm_->lowstate->dq, kArmJointCount);
            current_tau_ = read_arm_array(arm_->lowstate->tau, kArmJointCount);
            command_q_ = current_q_;
            command_dq_.assign(kArmJointCount, 0.0f);
            command_tau_.assign(kArmJointCount, 0.0f);
            last_state_observed_ = std::chrono::steady_clock::now();
            state_valid_ = true;
        }
        // Initialize with the currently measured position; never use zeros.
        arm_->setArmCmd(to_vec6(command_q_), Vec6::Zero(), Vec6::Zero());
        if (!arm_->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD))
            throw std::runtime_error("Z1 LOWCMD transition failed; abort startup");

        // CRITICAL: from this point control_loop is the sole sendRecv() owner.
        arm_->sendRecvThread->shutdown();
        running_.store(true);
        control_thread_ = std::thread(&Z1SdkController::control_loop, this);
        enabled_ = true;
        spdlog::info("Z1 SDK bridge armed: one sendRecv owner; BENCH ONLY");
    } catch (...) {
        running_.store(false);
        if (control_thread_.joinable()) control_thread_.join();
        if (arm_ && arm_->sendRecvThread) {
            // SDK background thread is only active during startup at this point.
            arm_->sendRecvThread->shutdown();
        }
        stop_controller_process();
        throw;
    }
}

Z1SdkController::~Z1SdkController()
{
    if (!enabled_) {
        return;
    }

    running_.store(false);
    if (control_thread_.joinable()) {
        control_thread_.join();
    }

    if (arm_) {
        // The custom communication thread has already stopped.
        // Restart vendor communication ONLY for an orderly FSM transition.
        // Never backToStart automatically; gravity/drop hazards need bench review.
        if (passive_on_shutdown_) {
            arm_->sendRecvThread->start();
            if (!arm_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE))
                spdlog::critical("Z1 PASSIVE shutdown transition failed");
            arm_->sendRecvThread->shutdown();
        }
    }

    stop_controller_process();
}

bool Z1SdkController::has_state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_valid_ && !sdk_fault_latched_ &&
           last_state_observed_ != std::chrono::steady_clock::time_point{} &&
           std::chrono::steady_clock::now() - last_state_observed_ <=
               std::chrono::milliseconds(100);
}

std::vector<float> Z1SdkController::current_q(const std::vector<float>& fallback) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_valid_) {
        return resize_command(fallback, kArmJointCount);
    }
    return current_q_;
}

std::vector<float> Z1SdkController::current_dq(size_t n) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto output = current_dq_;
    output.resize(n, 0.0f);
    return output;
}

std::vector<float> Z1SdkController::current_tau(size_t n) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto output = current_tau_;
    output.resize(n, 0.0f);
    return output;
}

std::vector<float> Z1SdkController::command_q(size_t n) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto output = command_q_;
    output.resize(n, 0.0f);
    return output;
}

void Z1SdkController::update(
    const std::vector<float>& target_q,
    const std::vector<float>& target_dq,
    const std::vector<float>& target_tau)
{
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    if (sdk_fault_latched_ || !state_valid_)
        throw std::runtime_error("Z1 SDK not ready or fault latched");
    if (target_q.size() != kArmJointCount ||
        (!target_dq.empty() && target_dq.size() != kArmJointCount) ||
        (!target_tau.empty() && target_tau.size() != kArmJointCount))
        throw std::invalid_argument("Z1 requires exactly six q (and optional six dq/tau)");
    for (float v: target_q) if (!std::isfinite(v))
        throw std::invalid_argument("Z1 nonfinite joint position target");
    for (float v: target_dq) if (!std::isfinite(v))
        throw std::invalid_argument("Z1 nonfinite joint velocity target");
    for (float v: target_tau) if (!std::isfinite(v))
        throw std::invalid_argument("Z1 nonfinite joint torque target");
    // No physical motion command is enabled by this M2.4b1 source patch.
    // Later M2.4b2 must integrate the validated per-mode hardware interlock.
    throw std::runtime_error("M2.4b1 Z1 motion update LOCKED until independent bench qualification");
}

void Z1SdkController::hold_current()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_valid_) {
        return;
    }
    command_q_ = current_q_;
    command_dq_.assign(kArmJointCount, 0.0f);
    command_tau_.assign(kArmJointCount, 0.0f);
}

void Z1SdkController::control_loop()
{
    using clock = std::chrono::steady_clock;
    auto wake_time = clock::now();
    const auto period = std::chrono::duration<double>(control_dt_sec_);

    while (running_.load()) {
        std::vector<float> command_q;
        std::vector<float> command_dq;
        std::vector<float> command_tau;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            command_q = command_q_;
            command_dq = command_dq_;
            command_tau = command_tau_;
        }

        arm_->setArmCmd(to_vec6(command_q), to_vec6(command_dq), to_vec6(command_tau));
        arm_->sendRecv();  // ONLY owning thread calls this while LOWCMD active
        // SDK's isDisConnect is a health flag, NOT a proven fresh-packet counter.
        bool valid = arm_->lowstate && arm_->_ctrlComp && arm_->_ctrlComp->udp &&
                     !arm_->_ctrlComp->udp->isDisConnect &&
                     arm_->lowstate->q.size() >= kArmJointCount &&
                     arm_->lowstate->dq.size() >= kArmJointCount &&
                     arm_->lowstate->tau.size() >= kArmJointCount &&
                     arm_->lowstate->errorstate.size() >= kArmJointCount &&
                     arm_->lowstate->isMotorConnected.size() >= kArmJointCount;
        if (valid) for (size_t i=0; i<kArmJointCount; ++i) {
            valid = valid && std::isfinite(arm_->lowstate->q[i]) &&
                std::isfinite(arm_->lowstate->dq[i]) &&
                std::isfinite(arm_->lowstate->tau[i]) &&
                (arm_->lowstate->errorstate[i] & static_cast<uint8_t>(~0x40u)) == 0 &&
                arm_->lowstate->isMotorConnected[i] == 0;
        }
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (valid) {
                current_q_ = read_arm_array(arm_->lowstate->q, kArmJointCount);
                current_dq_ = read_arm_array(arm_->lowstate->dq, kArmJointCount);
                current_tau_ = read_arm_array(arm_->lowstate->tau, kArmJointCount);
                state_valid_ = true;
                last_state_observed_ = clock::now();
            } else {
                state_valid_ = false;
                sdk_fault_latched_ = true;
            }
        }
        if (!valid) {
            spdlog::critical("Z1 motor/SDK health fault; stop software commands; operator E-stop required");
            running_.store(false);
            break;
        }

        wake_time += std::chrono::duration_cast<clock::duration>(period);
        std::this_thread::sleep_until(wake_time);
    }
}

void Z1SdkController::start_controller_process()
{
    if (!launch_controller_process_) {
        return;
    }

    const auto controller_binary = resolve_deploy_path(controller_binary_path_);
    if (!std::filesystem::exists(controller_binary)) {
        spdlog::warn("Z1 controller binary not found: {}", controller_binary.string());
        return;
    }

    const auto controller_dir = controller_binary.parent_path();
    controller_pid_ = fork();
    if (controller_pid_ == 0) {
        const auto controller_lib_dir = controller_dir / "lib";
        const auto ld_library_path = std::getenv("LD_LIBRARY_PATH");
        std::string new_ld_library_path = controller_lib_dir.string();
        if (ld_library_path && std::strlen(ld_library_path) > 0) {
            new_ld_library_path += ":" + std::string(ld_library_path);
        }
        setenv("LD_LIBRARY_PATH", new_ld_library_path.c_str(), 1);
        if (chdir(controller_dir.c_str()) != 0) {
            _exit(127);
        }
        execl(controller_binary.c_str(), controller_binary.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    if (controller_pid_ < 0) {
        spdlog::warn("Failed to fork z1_ctrl process.");
        controller_pid_ = -1;
        return;
    }

    spdlog::info("Started z1_ctrl helper process: pid={}", controller_pid_);
    std::this_thread::sleep_for(std::chrono::duration<double>(controller_startup_delay_sec_));
}

void Z1SdkController::stop_controller_process()
{
    if (controller_pid_ <= 0) {
        return;
    }

    const int pid = controller_pid_;
    controller_pid_ = -1;
    kill(pid, SIGINT);
    std::this_thread::sleep_for(std::chrono::duration<double>(controller_shutdown_delay_sec_));
    int status = 0;
    waitpid(pid, &status, 0);
    spdlog::info("Stopped z1_ctrl helper process: pid={}", pid);
}
