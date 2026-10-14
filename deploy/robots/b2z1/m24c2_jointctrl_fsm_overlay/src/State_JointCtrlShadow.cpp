#include "State_JointCtrlShadow.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>
#include <spdlog/spdlog.h>

namespace {
constexpr auto kPolicyDt = std::chrono::milliseconds(20);
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
std::array<float,3> xyz_from_fk(const HomoMat& T,
                                const std::array<float,3>& mount,
                                double tool_offset) {
    Eigen::Vector3d t = T.block<3,1>(0,3)
        + T.block<3,3>(0,0) * Eigen::Vector3d(tool_offset,0.,0.);
    return {static_cast<float>(mount[0]+t.x()),
            static_cast<float>(mount[1]+t.y()),
            static_cast<float>(mount[2]+t.z())};
}
Vec6 vec6(const std::array<float,6>& a) {
    Vec6 v=Vec6::Zero();
    for(int i=0;i<6;++i) v[i]=a[i];
    return v;
}
}

State_JointCtrlShadow::State_JointCtrlShadow(int state, std::string name)
    : FSMState(state, std::move(name)) {
    // This state is eagerly constructed by CtrlFSM even when inactive.
    // Force explicit loopback/sim configuration before allowing registration.
    const char* ack=std::getenv("B2Z1_SIM_ONLY_ACK");
    require(ack && std::string(ack)=="MUJOCO_LOOPBACK_NO_HARDWARE",
            "M2.4c2 simulation-only acknowledgement missing");
    const auto s=param::config["jointctrl_shadow"];
    require(s && s["sim_only"] && s["sim_only"].as<bool>(),
            "M2.4c2 jointctrl_shadow.sim_only=true is required");
    const auto interactive=param::config["interactive"];
    require(interactive && interactive["sim_only"] && interactive["sim_only"].as<bool>(),
            "M2.4c2 interactive.sim_only=true is required");
    const auto legacy=param::config["z1_sdk"];
    require(!legacy || !legacy["enabled"] || !legacy["enabled"].as<bool>(),
            "M2.4c2 forbids legacy LOWCMD SDK bridge");
    const auto real=param::config["z1_jointctrl"];
    require(!real || !real["enabled"] || !real["enabled"].as<bool>(),
            "M2.4c2 forbids real Z1 SDK JOINTCTRL output");
    auto cfg=param::config["interactive_arm_ik"];
    require(cfg && cfg["sim_only"] && cfg["sim_only"].as<bool>(),
            "M2.4c2 requires M2.3 simulated IK configuration");
    const auto mount=cfg["mount_xyz_base"].as<std::vector<double>>();
    require(mount.size()==3,"M2.4c2 requires 3-D arm mount");
    for(int i=0;i<3;++i) {
        require(std::isfinite(mount[i]),"nonfinite arm mount");
        mount_base_[i]=static_cast<float>(mount[i]);
    }
    tcp_offset_x_=cfg["tcp_offset_sdk_x"].as<double>(.0882);
    require(std::abs(tcp_offset_x_-.0882)<1.e-4,"M2.4c2 TCP model mismatch");
    std::filesystem::path p=interactive["policy_path"].as<std::string>();
    if(p.is_relative()) p=param::proj_dir/p;
    require(std::filesystem::is_regular_file(p),"M2.4c2 ONNX model not found");
    actor_=std::make_unique<isaaclab::OrtRunner>(p.string());
    shadow_=std::make_unique<b2z1::m24c::ShadowBridge>(
        [this](const interactive_b2z1::Observation799& obs) {
            std::unordered_map<std::string,std::vector<float>> feed;
            feed.emplace("obs",std::vector<float>(obs.begin(),obs.end()));
            auto raw=actor_->act(std::move(feed));
            require(raw.size()==18,"M2.4c2 expected 18 policy actions");
            interactive_b2z1::Joint18 out{};
            std::copy(raw.begin(),raw.end(),out.begin());
            return out;
        });
    spdlog::info("[M24c2] JOINTCTRL FSM SHADOW registered; NO physical Z1 SDK path: {}",p.string());
}

State_JointCtrlShadow::Sample State_JointCtrlShadow::sample() const {
    Sample s;
    std::lock_guard<std::mutex> guard(lowstate->mutex_);
    const auto& m=lowstate->msg_;
    require(m.motor_state().size()>=20,"M2.4c2 needs MuJoCo 20 motor slots");
    for(int i=0;i<12;++i) {
        s.legs[i]=m.motor_state()[i].q();
        s.leg_dq[i]=m.motor_state()[i].dq();
    }
    for(int i=0;i<6;++i) {
        s.arm[i]=m.motor_state()[12+i].q();
        s.arm_dq[i]=m.motor_state()[12+i].dq();
    }
    for(int i=0;i<4;++i) s.quat[i]=m.imu_state().quaternion()[i];
    for(int i=0;i<3;++i) s.gyro[i]=m.imu_state().gyroscope()[i];
    s.tick=static_cast<std::uint32_t>(m.tick());
    return s;
}

void State_JointCtrlShadow::enter() {
    require(!lowstate->isTimeout(),"M2.4c2 MuJoCo LowState missing");
    auto s=sample();
    for(float v : s.legs) require(std::isfinite(v),"invalid simulated B2 q");
    for(float v : s.arm) require(std::isfinite(v),"invalid simulated Z1 q");
    leg_hold_=s.legs;
    arm_hold_=s.arm;
    ik_preview_q_=s.arm;
    entry_base_=xyz_from_fk(model_.forwardKinematics(vec6(s.arm)),mount_base_,tcp_offset_x_);
    goal_base_=entry_base_;
    shadow_->resetOffline();
    actor_->reset();
    frames_=0;
    last_tick_=s.tick;
    entered_=true;
    fault_=false;
    next_eval_=Clock::now();
    last_sim_advance_=next_eval_;
    publish_sim_hold();
    spdlog::info("[M24c2] SIM SHADOW entered: arm base TCP goal=[{:.3f},{:.3f},{:.3f}], NO JOINTCTRL output",
                 goal_base_[0],goal_base_[1],goal_base_[2]);
}

void State_JointCtrlShadow::update_ik_goal(const C3& proposal) {
    if(fault_ || !entered_) return;
    const double dx=proposal[0]-entry_base_[0];
    const double dy=proposal[1]-entry_base_[1];
    const double dz=proposal[2]-entry_base_[2];
    if(std::sqrt(dx*dx+dy*dy+dz*dz)>.03+1e-7) {
        spdlog::warn("[M24c2] preview IK goal beyond 3cm simulation limit");
        return;
    }
    const Vec6 seed=vec6(ik_preview_q_);
    HomoMat T=model_.forwardKinematics(seed);
    T.block<3,1>(0,3)=Eigen::Vector3d(proposal[0]-mount_base_[0],
                                      proposal[1]-mount_base_[1],
                                      proposal[2]-mount_base_[2])
                            -T.block<3,3>(0,0)*Eigen::Vector3d(tcp_offset_x_,0.,0.);
    Vec6 q=seed;
    if(!model_.inverseKinematics(T,seed,q,false)||!q.allFinite()) {
        spdlog::warn("[M24c2] simulated IK rejected: no finite solution");return;
    }
    const Vec6 delta=q-seed;
    if(delta.cwiseAbs().maxCoeff()>.65) return;
    const auto limits_lo=model_.getJointQMin();
    const auto limits_hi=model_.getJointQMax();
    if(limits_lo.size()!=6 || limits_hi.size()!=6) return;
    for(int i=0;i<6;++i)
        if(q[i]<limits_lo[i]+.015 || q[i]>limits_hi[i]-.015) return;
    const auto fk=model_.forwardKinematics(q);
    if((fk.block<3,1>(0,3)-T.block<3,1>(0,3)).norm()>.002) return;
    for(int i=0;i<6;++i) ik_preview_q_[i]=static_cast<float>(q[i]);
    goal_base_=proposal;
    spdlog::info("[M24c2] preview IK goal [{:.3f},{:.3f},{:.3f}] (NOT transmitted)",
        goal_base_[0],goal_base_[1],goal_base_[2]);
}

void State_JointCtrlShadow::process_sim_keyboard() {
    if(!keyboard || !keyboard->on_pressed) return;
    const std::string k=keyboard->key();
    C3 p=goal_base_;
    if(k=="i")p[0]+=.01f;
    else if(k=="k")p[0]-=.01f;
    else if(k=="j")p[1]+=.01f;
    else if(k=="h")p[1]-=.01f;
    else if(k=="u")p[2]+=.01f;
    else if(k=="n")p[2]-=.01f;
    else return;
    update_ik_goal(p);
}

void State_JointCtrlShadow::publish_sim_hold() {
    // This routine ONLY exists for MuJoCo loopback. It deliberately does NOT
    // publish either the policy action or preview IK goals to motor commands.
    auto& motors=lowcmd->msg_.motor_cmd();
    require(motors.size()>=20,"M2.4c2 needs 20 simulated LowCmd motors");
    for(int i=0;i<12;++i){
        motors[i].mode(1); motors[i].q(leg_hold_[i]); motors[i].dq(0.f);
        motors[i].kp(360.f); motors[i].kd(5.f); motors[i].tau(0.f);
    }
    for(int i=0;i<6;++i){
        auto& motor=motors[12+i];
        motor.mode(1); motor.q(arm_hold_[i]);motor.dq(0.f);
        motor.kp(90.f);motor.kd(6.f);motor.tau(0.f);
    }
}

void State_JointCtrlShadow::run() {
    if(!entered_ || fault_) return;
    try {
        require(!lowstate->isTimeout(),"M2.4c2 sim LowState timed out");
        const auto now=Clock::now();
        const auto s=sample();
        require(s.tick>=last_tick_,"M2.4c2 sim tick went backwards");
        const bool new_sim_frame=s.tick>last_tick_;
        if(new_sim_frame){last_tick_=s.tick;last_sim_advance_=now;}
        require(now-last_sim_advance_<std::chrono::milliseconds(700),
                "M2.4c2 simulated clock stalled");
        process_sim_keyboard();
        if(now<next_eval_ || !new_sim_frame) return;
        next_eval_=now+kPolicyDt; // skip missed cycles instead of bursting
        // Only because one strictly advancing MuJoCo synthetic tick contains
        // BOTH B2 + Z1 data may the mock 'new packet' flag be set true.
        // It is never a proof of a real Z1 UDP datagram.
        shadow_->onB2(s.legs,s.leg_dq,s.quat,s.gyro,now);
        shadow_->onZ1(s.arm,s.arm_dq,now,true);
        auto p=shadow_->preview(b2z1::m24c::Mode::Stand,{0.f,0.f,0.f},
                                entry_base_,goal_base_,ik_preview_q_,Clock::now());
        ++frames_;
        if(frames_%50==1) spdlog::info(
            "[M24c2] SIM SHADOW frames={} obs799={} ee=[{:.3f},{:.3f},{:.3f}] leg_preview0={:.4f} arm_goal1={:.4f} NO ACTUATION",
            frames_,p.observation.size(),p.ee_obs_xyz[0],p.ee_obs_xyz[1],p.ee_obs_xyz[2],
            p.leg_policy_action[0],p.z1_ik_goal_unqualified[0]);
        publish_sim_hold();
    }catch(const std::exception& e){latch(e.what());}
}

void State_JointCtrlShadow::post_run() {
    // Main FSM's DDS publisher is used ONLY with loopback and the sim-only gate.
    if(entered_ && !fault_) FSMState::post_run();
}

void State_JointCtrlShadow::exit() {
    entered_=false;
    // NO vendor SDK mode switch, PASSIVE transition, backToStart or sendRecv.
}

void State_JointCtrlShadow::latch(const char* why) noexcept {
    fault_=true;
    spdlog::critical("[M24c2] SIM SHADOW FAULT: {}. Publication stops.",why);
}
