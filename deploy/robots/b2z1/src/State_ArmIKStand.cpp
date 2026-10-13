#include "State_ArmIKStand.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <mutex>
#include <vector>
#include <spdlog/spdlog.h>

namespace {
constexpr std::chrono::milliseconds kPeriod{5};
const std::array<float,12> kStandLegs = {
    .1f,.8f,-1.5f, -.1f,.8f,-1.5f,
    .1f,1.f,-1.5f, -.1f,1.f,-1.5f
};
template <typename T>
T optional(const YAML::Node& n,const char* key,T fallback) {
    return n && n[key] ? n[key].as<T>() : fallback;
}
HomoMat tcp_offset(double delta_x) {
    HomoMat out=HomoMat::Identity();
    out(0,3)=delta_x;
    return out;
}
double roll_from_quat(const std::array<float,4>& q) {
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    return std::atan2(2*(w*x+y*z),1-2*(x*x+y*y));
}
double pitch_from_quat(const std::array<float,4>& q) {
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    return std::asin(std::clamp(2*(w*y-z*x),-1.,1.));
}
}

State_ArmIKStand::State_ArmIKStand(int state,std::string state_string)
    : FSMState(state,std::move(state_string)) {
    const char* ack=std::getenv("B2Z1_SIM_ONLY_ACK");
    if (!ack || std::string(ack)!="MUJOCO_LOOPBACK_NO_HARDWARE")
        throw std::runtime_error("M1.9b requires SIM-ONLY acknowledgement");
    const auto interactive=param::config["interactive"];
    if (!interactive || !optional<bool>(interactive,"sim_only",false))
        throw std::runtime_error("M1.9b requires interactive.sim_only=true");
    const auto cfg=param::config["arm_ik_stand"];
    if (!cfg || !optional<bool>(cfg,"sim_only",false))
        throw std::runtime_error("M1.9b requires arm_ik_stand.sim_only=true");
    if (optional<bool>(param::config["z1_sdk"],"enabled",true))
        throw std::runtime_error("M1.9b SIM forbids direct Z1 SDK UDP control");
    auto mount=cfg["mount_xyz_base"].as<std::vector<double>>();
    if (mount.size()!=3) throw std::runtime_error("mount_xyz_base must have three numbers");
    mount_xyz_base_=Vec3d(mount[0],mount[1],mount[2]);
    tcp_offset_sdk_x_=optional<double>(cfg,"tcp_offset_sdk_x",.0882);
    position_step_m_=optional<double>(cfg,"position_step_m",.01);
    max_goal_shift_m_=optional<double>(cfg,"max_goal_shift_m",.07);
    max_arm_step_rad_=optional<double>(cfg,"max_arm_step_rad_per_5ms",.004);
    max_ik_joint_jump_rad_=optional<double>(cfg,"max_ik_joint_jump_rad",.65);
    xyz_keys_enabled_=optional<bool>(cfg,"enable_xyz_keys",false);
    if (std::abs(tcp_offset_sdk_x_-.0882)>1e-4 || position_step_m_<=0 || position_step_m_>.01 ||
        max_goal_shift_m_>.10 || max_goal_shift_m_<=0 || max_arm_step_rad_>.006 ||
        max_arm_step_rad_<=0 || max_ik_joint_jump_rad_<=0 || max_ik_joint_jump_rad_>.65)
        throw std::runtime_error("M1.9b requires validated geometric and conservative motion limits");
    for (int i=0;i<3;i++) if(!std::isfinite(mount_xyz_base_[i]))
        throw std::runtime_error("invalid Z1 mount offset");
    spdlog::info("[M19b] ArmIKStand loaded, SIM ONLY, XYZ_KEYS={}, link00_mount_B=[{:.3f},{:.3f},{:.3f}] TCP_Bx={:.4f}",
        xyz_keys_enabled_,mount_xyz_base_[0],mount_xyz_base_[1],mount_xyz_base_[2],tcp_offset_sdk_x_);
}

State_ArmIKStand::Sample State_ArmIKStand::sample() const {
    Sample s{};
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& msg=lowstate->msg_;
    if(msg.motor_state().size()<18) throw std::runtime_error("18 motor states required");
    for(int i=0;i<12;i++){
        s.leg_q[i]=msg.motor_state()[i].q();
        s.leg_dq[i]=msg.motor_state()[i].dq();
    }
    for(int i=0;i<6;i++){
        s.arm_q[i]=msg.motor_state()[12+i].q();
        s.arm_dq[i]=msg.motor_state()[12+i].dq();
    }
    for(int i=0;i<4;i++) s.quat[i]=msg.imu_state().quaternion()[i];
    s.tick=static_cast<std::uint32_t>(msg.tick());
    return s;
}

double State_ArmIKStand::maxabs(const Vec6& v){return v.cwiseAbs().maxCoeff();}

void State_ArmIKStand::reset_goal_from_pose(const Vec6& q) {
    sdk_tool_pose_=model_.forwardKinematics(q);
    goal_xyz_base_=mount_xyz_base_ + (sdk_tool_pose_*tcp_offset(tcp_offset_sdk_x_)).block<3,1>(0,3);
    initial_xyz_base_=goal_xyz_base_;
    arm_cmd_=q;
    arm_goal_q_=q;
}

void State_ArmIKStand::enter() {
    if(lowstate->isTimeout()) throw std::runtime_error("M1.9b SIM LowState timed out");
    const auto s=sample();
    if(!guard_state(s)) throw std::runtime_error("M1.9b requires stable FixStand before entry");
    arm_cmd_=s.arm_q;
    reset_goal_from_pose(s.arm_q);
    leg_hold_=s.leg_q;
    publish_pending_=false;
    fault_latched_=false;
    rejected_goals_=control_steps_=0;
    prev_tick_=s.tick;
    next_control_=Clock::now();
    last_tick_wall_=next_control_;
    auto& motors=lowcmd->msg_.motor_cmd();
    if(motors.size()<20) throw std::runtime_error("M1.9b requires MuJoCo 20-slot LowCmd");
    for(auto& motor:motors) {
        motor.mode(1);motor.q(0);motor.dq(0);motor.tau(0);motor.kp(0);motor.kd(0);
    }
    set_motor_commands();
    spdlog::info("[M19b] ENTER ArmIKStand: initial TCP in B2 base=[{:.4f},{:.4f},{:.4f}] m; key_xyz_enabled={}; o=enter, f=FixStand; i/k=+/-X, j/h=+/-Y, u/n=+/-Z, t=recapture, SPACE=freeze",
        goal_xyz_base_.x(),goal_xyz_base_.y(),goal_xyz_base_.z(),xyz_keys_enabled_);
}

bool State_ArmIKStand::guard_state(const Sample& s) {
    double worst_leg_err=0.,worst_leg_speed=0.;
    for(int i=0;i<12;i++) {
        if(!std::isfinite(s.leg_q[i]) || !std::isfinite(s.leg_dq[i])) return false;
        worst_leg_err=std::max(worst_leg_err,std::abs(double(s.leg_q[i])-kStandLegs[i]));
        worst_leg_speed=std::max(worst_leg_speed,std::abs(double(s.leg_dq[i])));
    }
    if(worst_leg_err>.25 || worst_leg_speed>.5 || maxabs(s.arm_dq)>.5) return false;
    for(int i=0;i<6;i++) if(!std::isfinite(s.arm_q[i]) || !std::isfinite(s.arm_dq[i])) return false;
    for(float v:s.quat) if(!std::isfinite(v)) return false;
    if (std::abs(roll_from_quat(s.quat))>.25 || std::abs(pitch_from_quat(s.quat))>.25) return false;
    return true;
}

bool State_ArmIKStand::try_goal(const Vec3d& proposed_base) {
    if(!xyz_keys_enabled_ || fault_latched_) return false;
    if((proposed_base-initial_xyz_base_).norm()>max_goal_shift_m_+1e-10){
        spdlog::warn("[M19b] Reject goal: >{:.3f}m from initial TCP",max_goal_shift_m_);
        return false;
    }
    // Orientation held at entry. TCP and SDK tool have identical orientation;
    // TCP is +0.0882m along SDK tool local X, so inverse offset is required.
    HomoMat desired_sdk=sdk_tool_pose_;
    desired_sdk.block<3,1>(0,3)=proposed_base-mount_xyz_base_-
        sdk_tool_pose_.block<3,3>(0,0)*Vec3d(tcp_offset_sdk_x_,0.,0.);
    Vec6 q_ik=arm_goal_q_;
    const bool ok=model_.inverseKinematics(desired_sdk,arm_goal_q_,q_ik,false);
    if(!ok || !q_ik.allFinite()) {
        spdlog::warn("[M19b] Reject IK: no valid finite solution");
        return false;
    }
    const HomoMat solved=model_.forwardKinematics(q_ik);
    const double err=(solved.block<3,1>(0,3)-desired_sdk.block<3,1>(0,3)).norm();
    if(!std::isfinite(err) || err>.002 ||
        maxabs(q_ik-arm_goal_q_)>max_ik_joint_jump_rad_){
        spdlog::warn("[M19b] Reject IK: ok={} fk_error_m={:.5f} jump_rad={:.4f}",
            ok,err,maxabs(q_ik-arm_goal_q_));
        return false;
    }
    const auto mins=model_.getJointQMin(),maxs=model_.getJointQMax();
    if(mins.size()!=6 || maxs.size()!=6) throw std::runtime_error("SDK Z1 joint limits unavailable");
    for(int i=0;i<6;i++) if(q_ik[i]<mins[i]+.015 || q_ik[i]>maxs[i]-.015){
        spdlog::warn("[M19b] Reject IK joint limit at j{} q={:.4f}",i+1,q_ik[i]);
        return false;
    }
    goal_xyz_base_=proposed_base;
    arm_goal_q_=q_ik;
    spdlog::info("[M19b] TARGET B2_base XYZ=[{:.4f},{:.4f},{:.4f}], max_joint_delta={:.4f}, FK_err={:.6f}",
        goal_xyz_base_.x(),goal_xyz_base_.y(),goal_xyz_base_.z(),maxabs(arm_goal_q_-arm_cmd_),err);
    return true;
}

void State_ArmIKStand::handle_key(const std::string& key,const Sample& s){
    if(key=="t") {
        if(maxabs(s.arm_dq)<.15){
            reset_goal_from_pose(s.arm_q);
            spdlog::info("[M19b] T: capture measured TCP as new local reference");
        }else spdlog::warn("[M19b] T denied: Z1 is moving");
        return;
    }
    if(key==" ") {
        // Freeze at currently commanded pose, not a discontinuous measured pose.
        arm_goal_q_=arm_cmd_;
        sdk_tool_pose_=model_.forwardKinematics(arm_cmd_);
        goal_xyz_base_=mount_xyz_base_+(sdk_tool_pose_*tcp_offset(tcp_offset_sdk_x_)).block<3,1>(0,3);
        spdlog::info("[M19b] SPACE: stop arm trajectory at last commanded pose");
        return;
    }
    Vec3d proposed=goal_xyz_base_;
    if(key=="i") proposed.x()+=position_step_m_;
    else if(key=="k") proposed.x()-=position_step_m_;
    else if(key=="j") proposed.y()+=position_step_m_;
    else if(key=="h") proposed.y()-=position_step_m_;
    else if(key=="u") proposed.z()+=position_step_m_;
    else if(key=="n") proposed.z()-=position_step_m_;
    else return;
    if(!try_goal(proposed)) ++rejected_goals_;
}

void State_ArmIKStand::set_motor_commands(){
    auto& motors=lowcmd->msg_.motor_cmd();
    for(int i=0;i<12;i++){
        motors[i].mode(1);motors[i].q(leg_hold_[i]);motors[i].dq(0);
        motors[i].tau(0);motors[i].kp(leg_kp_);motors[i].kd(leg_kd_);
    }
    for(int i=0;i<6;i++){
        auto& m=motors[12+i];m.mode(1);m.q(static_cast<float>(arm_cmd_[i]));
        m.dq(0);m.tau(0);m.kp(arm_kp_);m.kd(arm_kd_);
    }
}

void State_ArmIKStand::run(){
    try{
        if(lowstate->isTimeout()) throw std::runtime_error("SIM LowState timeout");
        const auto s=sample();
        const auto now=Clock::now();
        if(s.tick<prev_tick_) throw std::runtime_error("SIM tick moved backwards; reload scene");
        if(s.tick>prev_tick_){ prev_tick_=s.tick;last_tick_wall_=now; }
        if(now-last_tick_wall_>std::chrono::milliseconds(700))
            throw std::runtime_error("MuJoCo clock stalled 700ms");
        if(keyboard && keyboard->on_pressed) handle_key(keyboard->key(),s);
        if(now<next_control_)return;
        if(now>next_control_+kPeriod)next_control_=now;
        next_control_+=kPeriod;
        if(!fault_latched_){
            // Guard large deviations but do not abandon B2 support when arm IK fails.
            double arm_tracking=0;
            for(int i=0;i<6;i++) arm_tracking=std::max(arm_tracking,std::abs(double(s.arm_q[i])-arm_cmd_[i]));
            if(arm_tracking>.65 || std::abs(roll_from_quat(s.quat))>.25 ||
               std::abs(pitch_from_quat(s.quat))>.25){
                fault_latched_=true;
                arm_goal_q_=arm_cmd_;
                spdlog::error("[M19b] MOTION LOCKED tracking_err={:.3f} roll={:.3f} pitch={:.3f}. B2 hold remains active; press f to return FixStand",
                    arm_tracking,roll_from_quat(s.quat),pitch_from_quat(s.quat));
            }else {
                for(int i=0;i<6;i++)
                    arm_cmd_[i]+=std::clamp(arm_goal_q_[i]-arm_cmd_[i],-max_arm_step_rad_,max_arm_step_rad_);
            }
        }
        set_motor_commands();
        publish_pending_=true;
        ++control_steps_;
        if(control_steps_%200==0){
            // Read-only diagnostics: distinguish a valid IK target from the actual
            // TCP displacement produced by the simulated position-controlled arm.
            // All positions are in the B2 base frame; no motor commands change here.
            const Vec3d actual_tcp=(model_.forwardKinematics(s.arm_q)*tcp_offset(tcp_offset_sdk_x_)).block<3,1>(0,3)+mount_xyz_base_;
            const Vec3d commanded_tcp=(model_.forwardKinematics(arm_cmd_)*tcp_offset(tcp_offset_sdk_x_)).block<3,1>(0,3)+mount_xyz_base_;
            const Vec3d actual_delta=actual_tcp-initial_xyz_base_;
            const Vec3d goal_delta=goal_xyz_base_-initial_xyz_base_;
            const Vec3d xyz_error=actual_tcp-goal_xyz_base_;
            const Vec6 joint_error=s.arm_q-arm_cmd_;
            spdlog::info("[M19bdiag] tick={} xyz_keys={} tcp_tracking_err_m={:.4f} joint_tracking_err_rad={:.4f} goal_xyz=[{:.3f},{:.3f},{:.3f}] rejected={} lock={}",
                s.tick,xyz_keys_enabled_,xyz_error.norm(),maxabs(joint_error),
                goal_xyz_base_.x(),goal_xyz_base_.y(),goal_xyz_base_.z(),rejected_goals_,fault_latched_);
            spdlog::info("[M19btrack] actual_xyz=[{:.4f},{:.4f},{:.4f}] cmd_xyz=[{:.4f},{:.4f},{:.4f}] actual_delta=[{:.4f},{:.4f},{:.4f}] goal_delta=[{:.4f},{:.4f},{:.4f}] err_xyz=[{:.4f},{:.4f},{:.4f}] slew_q_lag={:.4f} qerr=[{:.3f},{:.3f},{:.3f},{:.3f},{:.3f},{:.3f}]",
                actual_tcp.x(),actual_tcp.y(),actual_tcp.z(),
                commanded_tcp.x(),commanded_tcp.y(),commanded_tcp.z(),
                actual_delta.x(),actual_delta.y(),actual_delta.z(),
                goal_delta.x(),goal_delta.y(),goal_delta.z(),
                xyz_error.x(),xyz_error.y(),xyz_error.z(),
                maxabs(arm_goal_q_-arm_cmd_),
                joint_error[0],joint_error[1],joint_error[2],joint_error[3],joint_error[4],joint_error[5]);
        }
    }catch(const std::exception& e){
        if(!fault_latched_)spdlog::error("[M19b] Loop fault: {} (hold last command, SIM only)",e.what());
        fault_latched_=true;
        set_motor_commands();
        publish_pending_=true;
    }
}

void State_ArmIKStand::post_run(){
    if(publish_pending_){publish_pending_=false;FSMState::post_run();}
}

void State_ArmIKStand::exit(){
    publish_pending_=false;
    arm_goal_q_=arm_cmd_;
    spdlog::info("[M19b] Exit ArmIKStand, return pose ownership to FSM");
}
