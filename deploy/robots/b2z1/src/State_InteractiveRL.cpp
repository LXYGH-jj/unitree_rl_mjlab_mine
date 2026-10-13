#include "State_InteractiveRL.h"

// Reuse exact M1.5n state preparation, output mapping, joint clamp and slew.
#include "joint_target_guard.h"
#include "interactive_feedback_adapter.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <spdlog/spdlog.h>

namespace {
using namespace interactive_b2z1;
using namespace interactive_b2z1::sim_control;
constexpr auto kControlPeriod = std::chrono::milliseconds(5); // M1.5n: 200 Hz
constexpr std::array<float, 3> kEEGoalBase{.45f, 0.f, .35f};
constexpr Joint12 kStandMotor = {
     .1f,.8f,-1.5f, -.1f,.8f,-1.5f,
     .1f,1.f,-1.5f, -.1f,1.f,-1.5f
};

template <typename T>
T cfg_or(const YAML::Node& cfg, const char* key, T fallback) {
    return (!cfg || !cfg[key] || cfg[key].IsNull()) ? fallback : cfg[key].as<T>();
}
} // namespace

State_InteractiveRL::State_InteractiveRL(int state, std::string state_string)
    : FSMState(state, std::move(state_string)) {
    const char* ack = std::getenv("B2Z1_SIM_ONLY_ACK");
    if (!ack || std::string(ack) != "MUJOCO_LOOPBACK_NO_HARDWARE")
        throw std::runtime_error("M1.8 SIM ACK missing: export B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE");
    const auto cfg = param::config["interactive"];
    if (!cfg || !cfg["sim_only"] || !cfg["sim_only"].as<bool>()) {
        throw std::runtime_error("M1.8 Interactive is SIM-ONLY (interactive.sim_only=true required)");
    }
    kp_ = cfg_or<float>(cfg, "b2_kp", kp_);
    kd_ = cfg_or<float>(cfg, "b2_kd", kd_);
    arm_kp_ = cfg_or<float>(cfg, "z1_kp", arm_kp_);
    arm_kd_ = cfg_or<float>(cfg, "z1_kd", arm_kd_);
    warmup_ms_ = cfg_or<std::uint32_t>(cfg, "warmup_ms", warmup_ms_);
    max_forward_ms_ = cfg_or<std::uint32_t>(cfg, "max_forward_ms", max_forward_ms_);
    forward_vx_ = cfg_or<float>(cfg, "forward_vx", forward_vx_);
    // M1.5n used 0.008 rad before walking and 0.04 rad after starting gait.
    const float stand_rate = cfg_or<float>(cfg, "stand_slew_rad_s", 1.6f);
    const float walk_rate = cfg_or<float>(cfg, "walk_slew_rad_s", 8.f);
    stand_step_ = stand_rate * .005f;
    walk_step_ = walk_rate * .005f;
    if (std::abs(kp_ - 360.f) > .01f || std::abs(kd_ - 5.f) > .01f ||
        std::abs(arm_kp_ - 90.f) > .01f || std::abs(arm_kd_ - 6.f) > .01f ||
        std::abs(stand_step_ - .008f) > 1.e-5f ||
        std::abs(walk_step_ - .04f) > 1.e-5f ||
        std::abs(forward_vx_ - .12f) > 1.e-5f ||
        warmup_ms_ < 2000 || max_forward_ms_ < 1 || max_forward_ms_ > 3500) {
        throw std::runtime_error("M1.8 SIM config outside exact M1.5n baseline gains/slew/velocity");
    }
    std::filesystem::path path = cfg["policy_path"].as<std::string>();
    if (path.is_relative()) path = param::proj_dir / path;
    if (!std::filesystem::is_regular_file(path))
        throw std::runtime_error("Interactive ONNX missing: " + path.string());
    runner_ = std::make_unique<isaaclab::OrtRunner>(path.string());
    pipeline_ = std::make_unique<interactive_b2z1::InteractivePipeline>(
        [this](const Observation799& observation) -> Joint18 {
            std::unordered_map<std::string, std::vector<float>> feed;
            feed.emplace("obs", std::vector<float>(observation.begin(), observation.end()));
            const auto result = runner_->act(std::move(feed));
            if (result.size() != kDof)
                throw std::runtime_error("ONNX must return exactly 18 actions");
            Joint18 action{};
            std::copy(result.begin(), result.end(), action.begin());
            return action;
        });
    registered_checks.emplace_back(std::make_pair(
        [this]() -> bool { return fault_; }, FSMStringMap.right.at("Passive")));
    spdlog::info("M1.8 InteractiveRL SINGLE LOOP; 50Hz ONNX + 200Hz motor targets + 200Hz SDK2 publish (SIM lo only); model={}", path.string());
}

State_InteractiveRL::SimSample State_InteractiveRL::read_sample() const {
    SimSample s;
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& msg = lowstate->msg_;
    if (msg.motor_state().size() < 20)
        throw std::runtime_error("M1.8 requires 20-slot SIM LowState including 6 Z1 joints");
    for (std::size_t i=0; i<12; ++i) {
        s.leg_q[i] = msg.motor_state()[i].q();
        s.leg_dq[i] = msg.motor_state()[i].dq();
    }
    for (std::size_t i=0; i<6; ++i) {
        s.arm_q[i] = msg.motor_state()[12+i].q();
        s.arm_dq[i] = msg.motor_state()[12+i].dq();
    }
    for (std::size_t i=0; i<4; ++i) s.quat[i] = msg.imu_state().quaternion()[i];
    for (std::size_t i=0; i<3; ++i) s.gyro[i] = msg.imu_state().gyroscope()[i];
    s.tick_ms = static_cast<std::uint32_t>(msg.tick());
    return s;
}

void State_InteractiveRL::clear_motion() {
    velocity_cmd_ = {0.f,0.f,0.f};
    walk_active_ = false;
}

void State_InteractiveRL::set_fault(const std::string& why) noexcept {
    if (!fault_) spdlog::error("M1.8 InteractiveRL FAULT -> Passive [SIM ONLY]: {}", why);
    fault_ = true;
    publish_pending_ = false;
    clear_motion();
}

void State_InteractiveRL::enter() {
    if (lowstate->isTimeout())
        throw std::runtime_error("M1.8 cannot enter without fresh SIM LowState");
    const SimSample s = read_sample();
    checkSimFeedback(s.leg_q, s.leg_dq, s.quat);
    float max_q_error = 0.f, max_dq = 0.f;
    for (std::size_t i=0; i<12; ++i) {
        max_q_error = std::max(max_q_error, std::abs(s.leg_q[i]-kStandMotor[i]));
        max_dq = std::max(max_dq, std::abs(s.leg_dq[i]));
    }
    if (max_q_error > .25f || max_dq > .5f)
        throw std::runtime_error("M1.8 requires stable FixStand (leg target error<0.25, dq<0.5)");
    for (std::size_t i=0; i<6; ++i) {
        if (!std::isfinite(s.arm_q[i]) || !std::isfinite(s.arm_dq[i]))
            throw std::runtime_error("nonfinite Z1 SIM joint feedback");
    }
    if (std::abs(s.arm_q[1]) + std::abs(s.arm_q[2]) < .15f)
        throw std::runtime_error("Z1 home joint 2/3 near zero: check B2Z1 scene");

    pipeline_->reset();
    runner_->reset();
    snapshots_.reset();
    arm_hold_ = s.arm_q;
    commanded_ = s.leg_q;
    desired_ = makeLegTargets(Joint18{}).q_target_motor; // same M1.5n default
    prev_sim_tick_ = enter_sim_tick_ = s.tick_ms;
    walk_start_tick_ = s.tick_ms;
    motor_steps_ = missed_deadlines_ = 0;
    walk_rearm_ = walk_active_ = has_walked_ = false;
    policy_ready_ = false;
    fault_ = publish_pending_ = false;
    clear_motion();
    next_control_ = Clock::now();
    last_sim_advance_ = next_control_;
    // Preserve M1.5n simulated LowCmd shape, including mode=1 and unused slots.
    auto& motors = lowcmd->msg_.motor_cmd();
    if (motors.size() < 20) throw std::runtime_error("M1.8 requires 20-slot SIM LowCmd");
    for (std::size_t i=0; i<motors.size(); ++i) {
        motors[i].mode(1);
        motors[i].q(0.f); motors[i].dq(0.f); motors[i].kp(0.f);
        motors[i].kd(0.f); motors[i].tau(0.f);
    }
    for (std::size_t i=0; i<12; ++i) {
        motors[i].q(commanded_[i]); motors[i].kp(kp_); motors[i].kd(kd_);
    }
    for (std::size_t i=0; i<6; ++i) {
        motors[12+i].q(arm_hold_[i]); motors[12+i].kp(arm_kp_); motors[12+i].kd(arm_kd_);
    }
    spdlog::info("M1.8 entered. Fixed 5ms joint target/publish loop and every 4th control tick ONNX; W max {}ms; SPACE stop/rearm", max_forward_ms_);
}

void State_InteractiveRL::process_keyboard(std::uint32_t tick_ms) {
    const bool pressed = keyboard && keyboard->on_pressed;
    const std::string key = keyboard ? keyboard->key() : std::string();
    if (pressed && key == " ") {
        clear_motion();
        walk_rearm_ = false;
        spdlog::info("M1.8 SPACE: stop/rearm");
    } else if (pressed && (key == "w" || key == "W")) {
        if (tick_ms >= enter_sim_tick_ && tick_ms-enter_sim_tick_ >= warmup_ms_
            && !walk_rearm_ && !walk_active_ && policy_ready_) {
            velocity_cmd_ = {forward_vx_, 0.f, 0.f};
            has_walked_ = true;
            walk_active_ = true;
            walk_rearm_ = true;
            walk_start_tick_ = tick_ms;
            spdlog::info("M1.8 W vx={} at sim tick={}ms", forward_vx_, tick_ms);
        } else {
            spdlog::info("M1.8 W ignored: warmup/rearm/policy readiness");
        }
    }
    if (walk_active_ && tick_ms>=walk_start_tick_
        && tick_ms-walk_start_tick_>=max_forward_ms_) {
        clear_motion();
        spdlog::info("M1.8 {}ms forward limit: vx=0, SPACE to rearm", max_forward_ms_);
    }
}

void State_InteractiveRL::motor_step(const SimSample& sample, Clock::time_point /*now*/) {
    // Exact M1.5n action/feedback path, no background policy thread:
    // 200Hz read -> every 4 ticks updateB2/Z1 -> adapt -> ONNX -> makeLegTargets
    // -> slewMotorTargets(0.008/0.04) -> LowCmd motor array update.
    checkSimFeedback(sample.leg_q, sample.leg_dq, sample.quat);
    for (std::size_t i=0; i<6; ++i) {
        if (!std::isfinite(sample.arm_q[i]) || !std::isfinite(sample.arm_dq[i]))
            throw std::runtime_error("invalid Z1 SIM feedback");
    }
    if (std::abs(sample.arm_q[1]-arm_hold_[1])>.6f ||
        std::abs(sample.arm_q[2]-arm_hold_[2])>.6f)
        throw std::runtime_error("Z1 simulated joint deviated from locked pose");

    if ((motor_steps_ % 4u) == 0u) {
        const auto ts = Clock::now();
        snapshots_.updateB2(sample.leg_q, sample.leg_dq, sample.quat, sample.gyro, ts);
        snapshots_.updateZ1(sample.arm_q, sample.arm_dq, ts);
        const auto input=adaptFeedback(snapshots_.copy(velocity_cmd_,kEEGoalBase),Clock::now());
        const auto outcome=pipeline_->step(input.robot,input.command);
        const auto targets=makeLegTargets(outcome.clipped_action);
        if (targets.saturated > 4)
            throw std::runtime_error("more than four leg target saturations (M1.5n guard)");
        desired_=targets.q_target_motor;
        policy_ready_=true;
        if ((motor_steps_ % 200u) == 0u) {
            float lag=0.f;
            for (std::size_t i=0; i<12; ++i)
                lag=std::max(lag, std::abs(desired_[i]-commanded_[i]));
            spdlog::info("[M18diag] sim_ms={} cmd_vx={} policy_steps={} gait_phase={:.4f} sat={} slew_target_lag={:.4f} missed_deadlines={}",
                sample.tick_ms, velocity_cmd_[0], pipeline_->observationBuilder().stepCount(),
                pipeline_->observationBuilder().gaitPhase(), targets.saturated, lag, missed_deadlines_);
        }
    }
    // In M1.5n, after the FIRST walking command, keep 0.04 rad/5ms even during STOP.
    const float step=has_walked_ ? walk_step_ : stand_step_;
    commanded_=slewMotorTargets(desired_,commanded_,step);
    for (std::size_t i=0; i<12; ++i)
        if (std::abs(commanded_[i]-sample.leg_q[i]) > 1.f)
            throw std::runtime_error("M1.5n hard tracking guard exceeded 1.0rad");
    auto& motors=lowcmd->msg_.motor_cmd();
    for (std::size_t i=0; i<12; ++i) {
        motors[i].q(commanded_[i]); motors[i].dq(0.f); motors[i].tau(0.f);
        motors[i].kp(kp_); motors[i].kd(kd_);
    }
    for (std::size_t i=0; i<6; ++i) {
        motors[12+i].q(arm_hold_[i]); motors[12+i].dq(0.f); motors[12+i].tau(0.f);
        motors[12+i].kp(arm_kp_); motors[12+i].kd(arm_kd_);
    }
    ++motor_steps_;
    publish_pending_=true;
}

void State_InteractiveRL::run() {
    if (fault_) return;
    try {
        if (lowstate->isTimeout()) throw std::runtime_error("SIM LowState timeout");
        const auto now=Clock::now();
        const SimSample state=read_sample();
        if (state.tick_ms < prev_sim_tick_)
            throw std::runtime_error("SIM tick moved backwards");
        if (state.tick_ms > prev_sim_tick_) {
            prev_sim_tick_=state.tick_ms;
            last_sim_advance_=now;
        }
        if (now - last_sim_advance_ > std::chrono::milliseconds(700))
            throw std::runtime_error("MuJoCo SIM clock paused/stalled >700ms");
        // Process FSM keyboard each 1kHz call so keypresses are not lost while
        // the actual motor/update/publish loop runs at 200 Hz.
        process_keyboard(state.tick_ms);
        if (now < next_control_) return;
        // M1.5n used an absolute 5ms steady_clock schedule. We do not simulate
        // missing controls with bursts of target steps after a delayed callback.
        if (now > next_control_+std::chrono::milliseconds(5)) {
            ++missed_deadlines_;
            next_control_=now;
        }
        next_control_+=kControlPeriod;
        motor_step(state, now);
    } catch (const std::exception& e) {
        set_fault(e.what());
    }
}

void State_InteractiveRL::post_run() {
    // BaseState::post_run() is virtual. Only this Interactive state overrides it:
    // old FSM states retain the original 1kHz post_run publication behavior.
    if (publish_pending_ && !fault_) {
        publish_pending_=false;
        FSMState::post_run();
    }
}

void State_InteractiveRL::exit() {
    clear_motion();
    policy_ready_=false;
    publish_pending_=false;
    // Preserve native FSM ownership and measured-pose handoff; no local SDK2 shutdown.
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    auto& motors=lowcmd->msg_.motor_cmd();
    const auto& state=lowstate->msg_.motor_state();
    for (std::size_t i=0; i<18 && i<state.size(); ++i) {
        motors[i].q(state[i].q()); motors[i].dq(0.f); motors[i].tau(0.f);
    }
}
