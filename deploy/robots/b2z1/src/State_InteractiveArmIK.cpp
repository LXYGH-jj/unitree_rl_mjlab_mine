#include "State_InteractiveArmIK.h"

// Reuse exact M1.5n state preparation, output mapping, joint clamp and slew.
#include "joint_target_guard.h"
#include "interactive_feedback_adapter.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <limits>
#include <spdlog/spdlog.h>

namespace {
using namespace interactive_b2z1;
using namespace interactive_b2z1::sim_control;
constexpr auto kControlPeriod = std::chrono::milliseconds(5); // M1.5n: 200 Hz
// Keep M1.8's EE goal observation during initial A/B standing integration.
HomoMat tcp_offset(double x) {
    HomoMat t=HomoMat::Identity(); t(0,3)=x; return t;
}
double roll_from_quat(const std::array<float,4>& q) {
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    return std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
}
double pitch_from_quat(const std::array<float,4>& q) {
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    return std::asin(std::clamp(2*(w*y-z*x),-1.,1.));
}
constexpr Joint12 kStandMotor = {
     .1f,.8f,-1.5f, -.1f,.8f,-1.5f,
     .1f,1.f,-1.5f, -.1f,1.f,-1.5f
};

template <typename T>
T cfg_or(const YAML::Node& cfg, const char* key, T fallback) {
    return (!cfg || !cfg[key] || cfg[key].IsNull()) ? fallback : cfg[key].as<T>();
}
} // namespace

State_InteractiveArmIK::State_InteractiveArmIK(int state, std::string state_string)
    : FSMState(state, std::move(state_string)) {
    const char* ack = std::getenv("B2Z1_SIM_ONLY_ACK");
    if (!ack || std::string(ack) != "MUJOCO_LOOPBACK_NO_HARDWARE")
        throw std::runtime_error("M2.0a SIM ACK missing: export B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE");
    if (cfg_or<bool>(param::config["z1_sdk"],"enabled",true))
        throw std::runtime_error("M2.0a forbids direct Z1 UDP in SIM");
    const auto mix = param::config["interactive_arm_ik"];
    if (!mix || !cfg_or<bool>(mix,"sim_only",false))
        throw std::runtime_error("M2.0a interactive_arm_ik.sim_only=true required");
    enable_xyz_keys_ = cfg_or<bool>(mix,"enable_xyz_keys",false);
    enable_walking_ = cfg_or<bool>(mix,"enable_walking",false);
    const std::string ee_src=cfg_or<std::string>(mix,"ee_obs_source","baseline_fixed");
    if (ee_src!="baseline_fixed" && ee_src!="ik_goal")
        throw std::runtime_error("M2.0a ee_obs_source must be baseline_fixed or ik_goal");
    policy_ee_uses_arm_goal_=(ee_src=="ik_goal");
    position_step_m_=cfg_or<double>(mix,"position_step_m",.01);
    max_goal_shift_m_=cfg_or<double>(mix,"max_goal_shift_m",.03);
    max_arm_step_rad_=cfg_or<double>(mix,"max_arm_step_rad_per_5ms",.004);
    max_ik_joint_jump_rad_=cfg_or<double>(mix,"max_ik_joint_jump_rad",.65);
    tcp_offset_x_=cfg_or<double>(mix,"tcp_offset_sdk_x",.0882);
    if(position_step_m_<=0||position_step_m_>.01 || max_goal_shift_m_<=0||max_goal_shift_m_>.07 ||
       max_arm_step_rad_<=0||max_arm_step_rad_>.004 ||
       max_ik_joint_jump_rad_<=0||max_ik_joint_jump_rad_>.65 ||
       std::abs(tcp_offset_x_-.0882)>.0001)
       throw std::runtime_error("M2.0a motion parameters out of conservative bounds");
    const auto xyz=mix["mount_xyz_base"].as<std::vector<double>>();
    if(xyz.size()!=3)throw std::runtime_error("M2.0a mount_xyz_base requires XYZ");
    for(int i=0;i<3;++i) {
        if (!std::isfinite(xyz[i])) throw std::runtime_error("M2.0a nonfinite mount XYZ");
        mount_base_[i]=xyz[i];
    }
    const auto cfg = param::config["interactive"];
    if (!cfg || !cfg["sim_only"] || !cfg["sim_only"].as<bool>()) {
        throw std::runtime_error("M2.0a Interactive is SIM-ONLY (interactive.sim_only=true required)");
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
        throw std::runtime_error("M2.0a SIM config outside exact M1.5n baseline gains/slew/velocity");
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
    // Unlike M1.8, a SIM policy fault does not switch B2 legs to Passive.
    // Instead hold last known command and latch out any new motion commands.
    spdlog::info("[M20a] InteractiveArmIK 50Hz ONNX/200Hz LowCmd SIM ONLY; xyz={} walking={} ee_obs={} model={}",
        enable_xyz_keys_,enable_walking_,ee_src,path.string());
}

State_InteractiveArmIK::SimSample State_InteractiveArmIK::read_sample() const {
    SimSample s;
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& msg = lowstate->msg_;
    if (msg.motor_state().size() < 20)
        throw std::runtime_error("M2.0a requires 20-slot SIM LowState including 6 Z1 joints");
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

void State_InteractiveArmIK::clear_motion() {
    velocity_cmd_ = {0.f,0.f,0.f};
    walk_active_ = false;
}

void State_InteractiveArmIK::set_fault(const std::string& why) noexcept {
    if (!fault_) spdlog::error("[M20a] policy FAULT latched; freeze B2+Z1 targets in SIM: {}", why);
    fault_ = true;
    publish_pending_ = false;
    arm_motion_locked_ = true;
    clear_motion();
}

void State_InteractiveArmIK::enter() {
    if (lowstate->isTimeout())
        throw std::runtime_error("M2.0a cannot enter without fresh SIM LowState");
    const SimSample s = read_sample();
    checkSimFeedback(s.leg_q, s.leg_dq, s.quat);
    float max_q_error = 0.f, max_dq = 0.f;
    for (std::size_t i=0; i<12; ++i) {
        max_q_error = std::max(max_q_error, std::abs(s.leg_q[i]-kStandMotor[i]));
        max_dq = std::max(max_dq, std::abs(s.leg_dq[i]));
    }
    if (max_q_error > .25f || max_dq > .5f)
        throw std::runtime_error("M2.0a requires stable FixStand (leg target error<0.25, dq<0.5)");
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
    capture_arm_goal(s);
    commanded_ = s.leg_q;
    desired_ = makeLegTargets(Joint18{}).q_target_motor; // same M1.5n default
    prev_sim_tick_ = enter_sim_tick_ = s.tick_ms;
    walk_start_tick_ = s.tick_ms;
    motor_steps_ = missed_deadlines_ = 0;
    walk_rearm_ = walk_active_ = has_walked_ = false;
    policy_ready_ = false;
    fault_ = publish_pending_ = false;
    arm_motion_locked_ = false;
    rejected_goals_ = 0;
    clear_motion();
    next_control_ = Clock::now();
    last_sim_advance_ = next_control_;
    // Preserve M1.5n simulated LowCmd shape, including mode=1 and unused slots.
    auto& motors = lowcmd->msg_.motor_cmd();
    if (motors.size() < 20) throw std::runtime_error("M2.0a requires 20-slot SIM LowCmd");
    for (std::size_t i=0; i<motors.size(); ++i) {
        motors[i].mode(1);
        motors[i].q(0.f); motors[i].dq(0.f); motors[i].kp(0.f);
        motors[i].kd(0.f); motors[i].tau(0.f);
    }
    for (std::size_t i=0; i<12; ++i) {
        motors[i].q(commanded_[i]); motors[i].kp(kp_); motors[i].kd(kd_);
    }
    for (std::size_t i=0; i<6; ++i) {
        motors[12+i].q(static_cast<float>(arm_cmd_[i])); motors[12+i].kp(arm_kp_); motors[12+i].kd(arm_kd_);
    }
    spdlog::info("[M20a] ENTER: B2 Interactive standing+Z1 IK state. xyz={} walking={}. GoalXYZ_B=[{:.4f},{:.4f},{:.4f}]",
        enable_xyz_keys_,enable_walking_,arm_goal_xyz_base_.x(),arm_goal_xyz_base_.y(),arm_goal_xyz_base_.z());
}

Vec6 State_InteractiveArmIK::as_vec6(const Arm6& arm) {
    Vec6 q=Vec6::Zero(); for(int i=0;i<6;++i)q[i]=arm[i]; return q;
}
double State_InteractiveArmIK::max_abs(const Vec6& v){return v.cwiseAbs().maxCoeff();}

void State_InteractiveArmIK::capture_arm_goal(const SimSample& sample) {
    arm_cmd_=as_vec6(sample.arm_q);
    arm_goal_q_=arm_cmd_;
    arm_entry_tool_=arm_model_.forwardKinematics(arm_cmd_);
    const HomoMat T=arm_entry_tool_*tcp_offset(tcp_offset_x_);
    arm_goal_xyz_base_=mount_base_+T.block<3,1>(0,3);
    arm_initial_xyz_base_=arm_goal_xyz_base_;
}

bool State_InteractiveArmIK::try_arm_goal(const Eigen::Vector3d& xyz) {
    if (!enable_xyz_keys_ || arm_motion_locked_ || fault_) return false;
    if((xyz-arm_initial_xyz_base_).norm()>max_goal_shift_m_+1e-10) {
        spdlog::warn("[M20a] Reject IK goal: workspace sphere >{:.3f}m",max_goal_shift_m_);
        return false;
    }
    HomoMat t=arm_entry_tool_;  // fixed TCP orientation throughout trajectory
    t.block<3,1>(0,3)=xyz-mount_base_-arm_entry_tool_.block<3,3>(0,0)*Eigen::Vector3d(tcp_offset_x_,0,0);
    Vec6 q=arm_goal_q_;
    const bool solved=arm_model_.inverseKinematics(t,arm_goal_q_,q,false);
    if(!solved||!q.allFinite()) {
        spdlog::warn("[M20a] Reject IK goal: no finite solution");return false;
    }
    const HomoMat fk=arm_model_.forwardKinematics(q);
    const double fk_err=(fk.block<3,1>(0,3)-t.block<3,1>(0,3)).norm();
    if(!std::isfinite(fk_err)||fk_err>.002||max_abs(q-arm_goal_q_)>max_ik_joint_jump_rad_) {
        spdlog::warn("[M20a] Reject IK goal: fk_err={} joint_jump={}",fk_err,max_abs(q-arm_goal_q_));return false;
    }
    const auto qmin=arm_model_.getJointQMin(), qmax=arm_model_.getJointQMax();
    if(qmin.size()!=6||qmax.size()!=6) throw std::runtime_error("Z1 SDK joint limits unavailable");
    for(int i=0;i<6;++i) {
        if(q[i]<qmin[i]+.015||q[i]>qmax[i]-.015) {
            spdlog::warn("[M20a] Reject IK: joint {} out of range",i+1);return false;
        }
    }
    arm_goal_q_=q;
    arm_goal_xyz_base_=xyz;
    spdlog::info("[M20a] IK TARGET XYZ_B=[{:.4f},{:.4f},{:.4f}] fk_err={:.6f}",xyz.x(),xyz.y(),xyz.z(),fk_err);
    return true;
}

void State_InteractiveArmIK::arm_key(const std::string& key) {
    Eigen::Vector3d proposed=arm_goal_xyz_base_;
    if(key=="i")proposed.x()+=position_step_m_;
    else if(key=="k")proposed.x()-=position_step_m_;
    else if(key=="j")proposed.y()+=position_step_m_;
    else if(key=="h")proposed.y()-=position_step_m_;
    else if(key=="u")proposed.z()+=position_step_m_;
    else if(key=="n")proposed.z()-=position_step_m_;
    else return;
    if(!try_arm_goal(proposed))++rejected_goals_;
}

void State_InteractiveArmIK::process_keyboard(std::uint32_t tick_ms) {
    const bool pressed=keyboard && keyboard->on_pressed;
    const std::string key=keyboard?keyboard->key():std::string();
    if(pressed && key==" ") {
        clear_motion();
        walk_rearm_=false;
        arm_goal_q_=arm_cmd_; // freeze at last commanded pose
        const HomoMat t=arm_model_.forwardKinematics(arm_cmd_)*tcp_offset(tcp_offset_x_);
        arm_goal_xyz_base_=mount_base_+t.block<3,1>(0,3);
        spdlog::info("[M20a] SPACE: B2 stop / arm freeze; rearm walking");
    } else if(pressed && (key=="i"||key=="k"||key=="j"||key=="h"||key=="u"||key=="n")) {
        if(enable_xyz_keys_ && policy_ready_ && tick_ms>=enter_sim_tick_ && tick_ms-enter_sim_tick_>=warmup_ms_)
            arm_key(key);
        else spdlog::warn("[M20a] Arm key blocked (disabled or warmup)");
    } else if(pressed && (key=="w"||key=="W")) {
        if(!enable_walking_) {
            spdlog::warn("[M20a] W blocked: enable_walking=false");
        } else if(tick_ms>=enter_sim_tick_ && tick_ms-enter_sim_tick_>=warmup_ms_ &&
                  !walk_rearm_ && !walk_active_ && policy_ready_ && !arm_motion_locked_ && !fault_) {
            velocity_cmd_={forward_vx_,0.f,0.f}; has_walked_=true;
            walk_active_=true;walk_rearm_=true;walk_start_tick_=tick_ms;
            spdlog::info("[M20a] W vx={} at sim tick={}ms",forward_vx_,tick_ms);
        } else spdlog::warn("[M20a] W blocked by readiness/arm lock/rearm");
    }
    if(walk_active_ && tick_ms>=walk_start_tick_ && tick_ms-walk_start_tick_>=max_forward_ms_) {
        clear_motion();
        spdlog::info("[M20a] max forward {}ms reached; vx=0",max_forward_ms_);
    }
}

void State_InteractiveArmIK::motor_step(const SimSample& sample, Clock::time_point /*now*/) {
    // Exact M1.5n action/feedback path, no background policy thread:
    // 200Hz read -> every 4 ticks updateB2/Z1 -> adapt -> ONNX -> makeLegTargets
    // -> slewMotorTargets(0.008/0.04) -> LowCmd motor array update.
    checkSimFeedback(sample.leg_q, sample.leg_dq, sample.quat);
    for (std::size_t i=0; i<6; ++i) {
        if (!std::isfinite(sample.arm_q[i]) || !std::isfinite(sample.arm_dq[i]))
            throw std::runtime_error("invalid Z1 SIM feedback");
    }
    const Vec6 arm_measured=as_vec6(sample.arm_q);
    const double arm_tracking=max_abs(arm_measured-arm_cmd_);
    const double roll=roll_from_quat(sample.quat),pitch=pitch_from_quat(sample.quat);
    if(arm_tracking>.65 || std::abs(roll)>.25 || std::abs(pitch)>.25) {
        if(!arm_motion_locked_) spdlog::error("[M20a] ARM MOTION LOCK tracking={:.3f} roll={:.3f} pitch={:.3f}",arm_tracking,roll,pitch);
        arm_motion_locked_=true;
        arm_goal_q_=arm_cmd_;
        clear_motion();
    }

    if ((motor_steps_ % 4u) == 0u) {
        const auto ts = Clock::now();
        snapshots_.updateB2(sample.leg_q, sample.leg_dq, sample.quat, sample.gyro, ts);
        snapshots_.updateZ1(sample.arm_q, sample.arm_dq, ts);
        const Command3 ee_goal=policy_ee_uses_arm_goal_
            ? Command3{float(arm_goal_xyz_base_.x()),float(arm_goal_xyz_base_.y()),float(arm_goal_xyz_base_.z())}
            : Command3{float(ee_baseline_xyz_.x()),float(ee_baseline_xyz_.y()),float(ee_baseline_xyz_.z())};
        const auto input=adaptFeedback(snapshots_.copy(velocity_cmd_,ee_goal),Clock::now());
        const auto outcome=pipeline_->step(input.robot,input.command);
        const auto targets=makeLegTargets(outcome.clipped_action);
        if (targets.saturated > 4)
            throw std::runtime_error("more than four leg target saturations (M1.5n guard)");
        desired_=targets.q_target_motor;
        policy_ready_=true;
        if ((motor_steps_ % 200u) == 0u) {
            // M2.0b read-only shadow: compare the two EE-goal observation modes.
            // NEVER alter `ee_goal`, `input.robot`, `outcome` or actuator commands.
            // Note: both goals are B2-base XYZ, while the policy observes EE-local
            // XYZ relative to the TRAINING nominal, yaw-aligned arm base offset.
            const FeedbackAdapterConfig training_frame{};
            const Command3 shadow_fixed_base{
                static_cast<float>(ee_baseline_xyz_.x()),
                static_cast<float>(ee_baseline_xyz_.y()),
                static_cast<float>(ee_baseline_xyz_.z())};
            const Command3 shadow_ik_base{
                static_cast<float>(arm_goal_xyz_base_.x()),
                static_cast<float>(arm_goal_xyz_base_.y()),
                static_cast<float>(arm_goal_xyz_base_.z())};
            const Command3 shadow_fixed_local = eeGoalObservationFromBaseTarget(
                shadow_fixed_base, sample.quat, training_frame.nominal_arm_offset_yaw_frame);
            const Command3 shadow_ik_local = eeGoalObservationFromBaseTarget(
                shadow_ik_base, sample.quat, training_frame.nominal_arm_offset_yaw_frame);
            const double dx = double(shadow_ik_local[0]) - double(shadow_fixed_local[0]);
            const double dy = double(shadow_ik_local[1]) - double(shadow_fixed_local[1]);
            const double dz = double(shadow_ik_local[2]) - double(shadow_fixed_local[2]);
            const auto& obs799 = outcome.observation;
            constexpr std::size_t kLastHistoryEe = kProprioDim + kPrivDim
                + (kHistoryLen - 1) * kProprioDim + 60;
            spdlog::info(
                "[M20obs] active={} obs60_62=[{:.4f},{:.4f},{:.4f}] "
                "fixed_local=[{:.4f},{:.4f},{:.4f}] ik_local=[{:.4f},{:.4f},{:.4f}] "
                "delta_local=[{:.4f},{:.4f},{:.4f}] delta_norm={:.4f} "
                "hist_last_ee=[{:.4f},{:.4f},{:.4f}]",
                policy_ee_uses_arm_goal_ ? "ik_goal" : "baseline_fixed",
                obs799[60], obs799[61], obs799[62],
                shadow_fixed_local[0], shadow_fixed_local[1], shadow_fixed_local[2],
                shadow_ik_local[0], shadow_ik_local[1], shadow_ik_local[2],
                dx, dy, dz, std::sqrt(dx*dx + dy*dy + dz*dz),
                obs799[kLastHistoryEe], obs799[kLastHistoryEe+1],
                obs799[kLastHistoryEe+2]);
            // M2.0b.2 observation SNAPSHOT ONLY. Never infer twice inside the
            // 200Hz control callback. Offline analysis replays the exact 799
            // floats with an alternative EE target after simulation stops.
            // Layout: 824 little-endian float32 values per record:
            // [sim_tick_ms, fixed_local_xyz(3), ik_local_xyz(3), obs799, raw_action18].
            // Capture is enabled only for SIM standing with baseline_fixed.
            if (!policy_ee_uses_arm_goal_ && !enable_walking_) {
                std::array<float, 1 + 3 + 3 + kObsDim + kDof> record{};
                std::size_t at=0;
                record[at++]=static_cast<float>(sample.tick_ms);
                for (float v:shadow_fixed_local) record[at++]=v;
                for (float v:shadow_ik_local) record[at++]=v;
                for (float v:obs799) record[at++]=v;
                for (float v:outcome.raw_action) record[at++]=v;
                const auto output=param::proj_dir / "build/m20b2_ee_shadow_snapshots.f32";
                std::ofstream dump(output, std::ios::binary | std::ios::app);
                if (dump) {
                    dump.write(reinterpret_cast<const char*>(record.data()),
                               static_cast<std::streamsize>(record.size()*sizeof(float)));
                }
                if (!dump.good()) {
                    spdlog::warn("[M20capture] Could not write {}",output.string());
                } else {
                    spdlog::info("[M20capture] sim_ms={} capture_bytes={} path={}",
                        sample.tick_ms,record.size()*sizeof(float),output.string());
                }
            }
            float lag=0.f;
            for (std::size_t i=0; i<12; ++i)
                lag=std::max(lag, std::abs(desired_[i]-commanded_[i]));
            const HomoMat actual_pose=arm_model_.forwardKinematics(arm_measured)*tcp_offset(tcp_offset_x_);
            const Eigen::Vector3d actual_xyz=mount_base_+actual_pose.block<3,1>(0,3);
            const double tcp_err=(actual_xyz-arm_goal_xyz_base_).norm();
            spdlog::info("[M20diag] sim_ms={} vx={} policy_steps={} phase={:.4f} sat={} leg_lag={:.4f} arm_track={:.4f} tcp_err_m={:.4f} arm_lock={} xyz={} rejected={} misses={}",
                sample.tick_ms,velocity_cmd_[0],pipeline_->observationBuilder().stepCount(),
                pipeline_->observationBuilder().gaitPhase(),targets.saturated,lag,arm_tracking,
                tcp_err,arm_motion_locked_,enable_xyz_keys_,rejected_goals_,missed_deadlines_);
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
    if(!arm_motion_locked_ && !fault_) {
        for(int i=0;i<6;++i)
            arm_cmd_[i]+=std::clamp(arm_goal_q_[i]-arm_cmd_[i],-max_arm_step_rad_,max_arm_step_rad_);
    }
    for (std::size_t i=0; i<6; ++i) {
        motors[12+i].q(static_cast<float>(arm_cmd_[i])); motors[12+i].dq(0.f); motors[12+i].tau(0.f);
        motors[12+i].kp(arm_kp_); motors[12+i].kd(arm_kd_);
    }
    ++motor_steps_;
    publish_pending_=true;
}

void State_InteractiveArmIK::write_motor_commands() {
    auto& motors=lowcmd->msg_.motor_cmd();
    for(std::size_t i=0;i<12;++i) {
        motors[i].mode(1);motors[i].q(commanded_[i]);motors[i].dq(0.f);motors[i].tau(0.f);
        motors[i].kp(kp_);motors[i].kd(kd_);
    }
    for(int i=0;i<6;++i) {
        auto& m=motors[12+i];m.mode(1);m.q(static_cast<float>(arm_cmd_[i]));
        m.dq(0.f);m.tau(0.f);m.kp(arm_kp_);m.kd(arm_kd_);
    }
}
void State_InteractiveArmIK::hold_on_policy_fault() {
    // SIM-only last-target PD hold. This is not a recovery controller.
    write_motor_commands();
    publish_pending_=true;
}
void State_InteractiveArmIK::run() {
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
        if(!fault_) process_keyboard(state.tick_ms);
        if (now < next_control_) return;
        // M1.5n used an absolute 5ms steady_clock schedule. We do not simulate
        // missing controls with bursts of target steps after a delayed callback.
        if (now > next_control_+std::chrono::milliseconds(5)) {
            ++missed_deadlines_;
            next_control_=now;
        }
        next_control_+=kControlPeriod;
        if(fault_)hold_on_policy_fault();
        else motor_step(state,now);
    } catch (const std::exception& e) {
        set_fault(e.what());
    }
}

void State_InteractiveArmIK::post_run() {
    // BaseState::post_run() is virtual. Only this Interactive state overrides it:
    // old FSM states retain the original 1kHz post_run publication behavior.
    if (publish_pending_ && !lowstate->isTimeout()) {
        publish_pending_=false;
        FSMState::post_run();
    }
}

void State_InteractiveArmIK::exit() {
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
