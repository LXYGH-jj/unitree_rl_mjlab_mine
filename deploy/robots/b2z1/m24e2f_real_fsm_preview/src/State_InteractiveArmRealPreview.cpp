#include "State_InteractiveArmRealPreview.h"
#include "interactive_observation.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <spdlog/spdlog.h>

namespace {
using namespace interactive_b2z1;
constexpr auto kPolicyPeriod=std::chrono::milliseconds(20);
HomoMat tcpOffset(double x) { HomoMat t=HomoMat::Identity(); t(0,3)=x; return t; }
std::uint32_t tickValue(const LowState_t& ls) {
    // Underlying B2 SDK2 DDS LowState.tick() must match the robot's packet counter.
    return static_cast<std::uint32_t>(ls.msg_.tick());
}
}  // namespace

State_InteractiveArmRealPreview::State_InteractiveArmRealPreview(int state,std::string label)
 : FSMState(state,std::move(label)) {
    const auto c=param::config["interactive_real_preview"];
    if (!c || !c["sim_only"] || !c["sim_only"].as<bool>() ||
        !c["backend"] || c["backend"].as<std::string>()!="mock")
        throw std::runtime_error("M2.4e2f PREVIEW requires sim_only=true, backend=mock");
    const auto vendor=param::config["z1_sdk"];
    if (vendor && vendor["enabled"] && vendor["enabled"].as<bool>())
        throw std::runtime_error("M2.4e2f refuses legacy Z1 SDK client");
    const auto diagnostics=param::config["z1_diagnostic"];
    if (diagnostics && diagnostics["enabled"] && diagnostics["enabled"].as<bool>())
        throw std::runtime_error("M2.4e2f refuses Z1 diagnostic client");
    const char* ack=std::getenv("B2Z1_SIM_ONLY_ACK");
    if (!ack || std::string(ack)!="MUJOCO_LOOPBACK_NO_HARDWARE")
        throw std::runtime_error("M2.4e2f SIM ACK missing");
    socket_path_=c["socket_path"].as<std::string>();
    if(socket_path_.empty() || socket_path_[0]!='/')
        throw std::runtime_error("M2.4e2f socket must have absolute path");
    use_ik_ee_observation_=c["ee_obs_source"] && c["ee_obs_source"].as<std::string>()=="ik_goal";
    enable_ik_preview_keys_=c["enable_ik_preview_keys"] && c["enable_ik_preview_keys"].as<bool>();
    if (c["tcp_offset_sdk_x"]) tcp_offset_x_=c["tcp_offset_sdk_x"].as<double>();
    if(!std::isfinite(tcp_offset_x_) || std::abs(tcp_offset_x_-.0882)>.001)
        throw std::runtime_error("M2.4e2f unexpected TCP offset");
    const auto xyz=c["mount_xyz_base"].as<std::vector<double>>();
    if(xyz.size()!=3)throw std::runtime_error("M2.4e2f bad mount XYZ");
    for(int i=0;i<3;++i) { if(!std::isfinite(xyz[i]))throw std::runtime_error("nonfinite mount"); mount_[i]=xyz[i]; }
    // No motion output in this preview state. Do not register it as a real control FSM.
    spdlog::info("M2.4e2f PREVIEW FSM constructed; no new B2/Z1 targets can be emitted");
}

void State_InteractiveArmRealPreview::enter() {
    const auto c=param::config["interactive_real_preview"];
    std::filesystem::path model=c["policy_path"].as<std::string>();
    if(model.is_relative())model=param::proj_dir/model;
    if(!std::filesystem::is_regular_file(model))throw std::runtime_error("M2.4e2f missing ONNX: "+model.string());
    runner_=std::make_unique<isaaclab::OrtRunner>(model.string());
    pipeline_=std::make_unique<InteractivePipeline>([this](const Observation799& obs)->Joint18 {
        std::unordered_map<std::string,std::vector<float>> feed;
        feed.emplace("obs",std::vector<float>(obs.begin(),obs.end()));
        const auto raw=runner_->act(std::move(feed));
        if(raw.size()!=18)throw std::runtime_error("ONNX action count not 18");
        Joint18 out{};
        std::copy(raw.begin(),raw.end(),out.begin());
        return out;
    });
    pipeline_->reset();runner_->reset();
    z1_.start(socket_path_,m24e2e::Mode::Mock);
    tick_seen_=false;ik_initialized_=false;faulted_=false;
    fault_reason_.clear();policy_steps_=0;cmd_={0,0,0};
    next_control_=entered_=Clock::now();
    spdlog::info("M2.4e2f entered; preview only; connected to mock socket {}",socket_path_);
}

m24e2e::B2State State_InteractiveArmRealPreview::captureB2(Clock::time_point now) {
    if(lowstate->isTimeout())throw std::runtime_error("B2 DDS lowstate timeout");
    m24e2e::B2State b;
    std::lock_guard<std::mutex> lock(lowstate->mutex_);
    const auto& msg=lowstate->msg_;
    if(msg.motor_state().size()<12)throw std::runtime_error("B2 motor_state below 12");
    for(std::size_t i=0;i<12;++i){b.motor_q[i]=msg.motor_state()[i].q();b.motor_dq[i]=msg.motor_state()[i].dq();}
    for(std::size_t i=0;i<4;++i)b.quat_wxyz[i]=msg.imu_state().quaternion()[i];
    for(std::size_t i=0;i<3;++i)b.gyro[i]=msg.imu_state().gyroscope()[i];
    const auto tick=tickValue(*lowstate);
    if(!tick_seen_){tick_seen_=true;last_tick_=tick;last_tick_first_seen_=now;}
    else if(tick!=last_tick_){
        // This is FIRST-SEEN time in FSM, not DDS callback arrival time.
        // Suitable ONLY for simulator preview; not a hardware freshness certificate.
        if(tick<last_tick_)throw std::runtime_error("B2 simulation tick moved backwards");
        last_tick_=tick;last_tick_first_seen_=now;
    }
    if(now-last_tick_first_seen_>std::chrono::milliseconds(100))
        throw std::runtime_error("B2 lowstate tick stalled");
    b.packet_received_at=last_tick_first_seen_;
    b.valid=true;
    return b;
}

void State_InteractiveArmRealPreview::previewIk(const m24e2e::ArmState& measured){
    Vec6 q;
    for(int i=0;i<6;++i)q[i]=measured.q[i];
    if(!ik_initialized_){
        arm_goal_joint_=q;
        arm_entry_tool_=arm_model_.forwardKinematics(q);
        const auto t=arm_entry_tool_*tcpOffset(tcp_offset_x_);
        entry_goal_xyz_=arm_goal_xyz_=mount_+t.block<3,1>(0,3);
        ik_initialized_=true;
        return;
    }
    if(!enable_ik_preview_keys_ || !keyboard || !keyboard->on_pressed)return;
    const std::string key=keyboard->key();
    Eigen::Vector3d goal=arm_goal_xyz_;
    if(key=="i")goal.x()+=.01;
    else if(key=="k")goal.x()-=.01;
    else if(key=="j")goal.y()+=.01;
    else if(key=="h")goal.y()-=.01;
    else if(key=="u")goal.z()+=.01;
    else if(key=="n")goal.z()-=.01;
    else return;
    if((goal-entry_goal_xyz_).norm()>.03) {spdlog::warn("M2.4e2f preview IK workspace reject");return;}
    HomoMat target=arm_entry_tool_;
    target.block<3,1>(0,3)=goal-mount_-
        arm_entry_tool_.block<3,3>(0,0)*Eigen::Vector3d(tcp_offset_x_,0.,0.);
    Vec6 solved=arm_goal_joint_;
    if(!arm_model_.inverseKinematics(target,arm_goal_joint_,solved,false)||!solved.allFinite()){
        spdlog::warn("M2.4e2f IK no solution");return;
    }
    const auto fk=arm_model_.forwardKinematics(solved);
    const auto qlo=arm_model_.getJointQMin(),qhi=arm_model_.getJointQMax();
    if(qlo.size()!=6||qhi.size()!=6||(fk.block<3,1>(0,3)-target.block<3,1>(0,3)).norm()>.002||
       (solved-arm_goal_joint_).cwiseAbs().maxCoeff()>.65) {spdlog::warn("M2.4e2f IK verify reject");return;}
    for(int i=0;i<6;++i)if(solved[i]<qlo[i]+.015 || solved[i]>qhi[i]-.015){
        spdlog::warn("M2.4e2f IK joint bounds reject");return;
    }
    arm_goal_joint_=solved;arm_goal_xyz_=goal;
    spdlog::info("M2.4e2f IK PREVIEW ONLY XYZ=[{:.3f},{:.3f},{:.3f}]",goal.x(),goal.y(),goal.z());
}

void State_InteractiveArmRealPreview::fault(const std::string& why) noexcept {
    if(!faulted_)spdlog::error("M2.4e2f preview fault: {}",why);
    faulted_=true;fault_reason_=why;
}
void State_InteractiveArmRealPreview::run(){
    if(faulted_)return;
    auto now=Clock::now();
    if(now<next_control_)return;
    if(now>next_control_+kPolicyPeriod)next_control_=now;
    next_control_+=kPolicyPeriod;
    try{
        auto b2=captureB2(now);
        m24e2e::ArmState z1;std::string err;
        if(!z1_.latest(z1,err)){
            if(now-entered_<std::chrono::seconds(2))return;
            throw std::runtime_error("Z1 IPC not ready: "+err);
        }
        previewIk(z1);
        const Command3 ee=use_ik_ee_observation_ && ik_initialized_ ?
            Command3{float(arm_goal_xyz_.x()),float(arm_goal_xyz_.y()),float(arm_goal_xyz_.z())}:
            fixed_goal_;
        const auto converted=m24e2e::makeLiveInput(b2,z1,cmd_,ee,now,m24e2e::Mode::Mock);
        const auto outcome=pipeline_->step(converted.robot,converted.command);
        ++policy_steps_;
        if(policy_steps_%50==1){
            spdlog::info("[M2.4e2f] 799D pipeline ok; steps={} a0={:.4f} a12={:.4f}; Z1 mock mode; NO NEW MOTOR OUTPUT",
                policy_steps_,outcome.clipped_action[0],outcome.clipped_action[12]);
        }
    }catch(const std::exception& e){fault(e.what());}
}
void State_InteractiveArmRealPreview::post_run(){
    // Intentionally does not publish new B2 targets. On real B2, loss of lowcmd
    // is NOT a qualified safe stop: this state is SIM-only, not a bench controller.
}
void State_InteractiveArmRealPreview::exit(){z1_.stop();runner_.reset();pipeline_.reset();}
