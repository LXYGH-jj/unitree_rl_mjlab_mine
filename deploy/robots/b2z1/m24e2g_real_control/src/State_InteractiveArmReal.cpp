#include "State_InteractiveArmReal.h"
#include "StampedB2LowState.h"
#include "joint_target_guard.h"
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
using namespace interactive_b2z1::sim_control;
constexpr auto kMotorPeriod=std::chrono::milliseconds(5);  // 200Hz B2 target loop
constexpr auto kOnnxDivisor=4u; // 50Hz
constexpr std::array<float,12> kStandMotor={.1f,.8f,-1.5f,-.1f,.8f,-1.5f,.1f,1.f,-1.5f,-.1f,1.f,-1.5f};
HomoMat tcpOffset(double x){ HomoMat t=HomoMat::Identity();t(0,3)=x;return t; }
// Unlike the old simulation states, never read Z1 from B2 LowState motor slots 12..17.
}

State_InteractiveArmReal::State_InteractiveArmReal(int state,std::string label)
 : FSMState(state,std::move(label)) {
    const auto c=param::config["interactive_real"];
    if(!c||!c["enabled"]||!c["enabled"].as<bool>())
        throw std::runtime_error("M2.4e2g real FSM must be explicitly enabled");
    const auto legacy=param::config["z1_sdk"];
    const auto diag=param::config["z1_diagnostic"];
    if((legacy && legacy["enabled"] && legacy["enabled"].as<bool>())||
       (diag && diag["enabled"] && diag["enabled"].as<bool>()))
        throw std::runtime_error("Old direct Z1 SDK/diagnostic ownership MUST be disabled");
    const std::string backend=c["z1_backend"].as<std::string>();
    if(backend=="sdk")mode_=m24e2e::Mode::SdkUnqualified;
    else if(backend=="mock")mode_=m24e2e::Mode::Mock;
    else throw std::runtime_error("invalid Z1 backend");
    socket_path_=c["socket_path"].as<std::string>();
    if(socket_path_.empty()||socket_path_[0]!='/')throw std::runtime_error("IPC socket must be absolute");
    send_b2_=c["enable_b2_output"].as<bool>(false);
    walk_enabled_=c["enable_walking"].as<bool>(false);
    if(c["enable_z1_sdk_targets"] && c["enable_z1_sdk_targets"].as<bool>())
        throw std::runtime_error("M2.4e2g Z1 SDK TARGETs not yet bench qualified (locked)");
    if(send_b2_){
        const char* ack=std::getenv("B2Z1_B2_ACTUATION_ACK");
        if(!ack||std::string(ack)!="B2_REAL_BENCH_CONFIRMED")
            throw std::runtime_error("B2 actuation requires separate supervised bench ACK");
        if(mode_!=m24e2e::Mode::SdkUnqualified)
            throw std::runtime_error("real B2 motor output requires real Z1 feedback mode");
    }
    const auto xyz=c["mount_xyz_base"].as<std::vector<double>>();
    if(xyz.size()!=3)throw std::runtime_error("bad mount XYZ");
    for(int i=0;i<3;++i){if(!std::isfinite(xyz[i]))throw std::runtime_error("nonfinite mount");mount_[i]=xyz[i];}
    tcp_offset_=c["tcp_offset_sdk_x"].as<double>(.0882);
    if(std::abs(tcp_offset_-.0882)>.001)throw std::runtime_error("unexpected TCP offset");
    use_ik_goal_=c["ee_obs_source"].as<std::string>("ik_goal")=="ik_goal";
    if(!send_b2_&&walk_enabled_)throw std::runtime_error("walking forbidden when B2 output disabled");
    // On a policy/feedback fault, promptly transition into the existing native
    // FixStand controller rather than silently cease periodic B2 LowCmd updates.
    // This is an engineering fallback, NOT a certified emergency stop.
    registered_checks.emplace_back(std::make_pair(
        [this]() -> bool { return faulted_; }, FSMStringMap.right.at("FixStand")));
    spdlog::info("M2.4e2g real FSM candidate constructed: Z1 feedback={}, B2 motor={}, Z1 target=LOCKED",
                 backend,send_b2_);
}

m24e2e::B2State State_InteractiveArmReal::captureB2() const {
    if(lowstate->isTimeout())throw std::runtime_error("B2 DDS timeout");
    auto* stamped=dynamic_cast<StampedB2LowState*>(lowstate.get());
    if(!stamped)throw std::runtime_error("B2 source lacks DDS callback reception timestamp");
    m24e2e::B2State b;
    std::lock_guard<std::mutex> lk(lowstate->mutex_);
    const auto& msg=lowstate->msg_;
    if(msg.motor_state().size()<12)throw std::runtime_error("B2 motor_state<12");
    for(std::size_t i=0;i<12;++i){b.motor_q[i]=msg.motor_state()[i].q();b.motor_dq[i]=msg.motor_state()[i].dq();}
    for(std::size_t i=0;i<4;++i)b.quat_wxyz[i]=msg.imu_state().quaternion()[i];
    for(std::size_t i=0;i<3;++i)b.gyro[i]=msg.imu_state().gyroscope()[i];
    b.packet_received_at=stamped->receivedAtLocked();
    b.valid=(b.packet_received_at!=Clock::time_point{});
    return b;
}
void State_InteractiveArmReal::enter(){
    if(lowstate->isTimeout())throw std::runtime_error("B2 not connected");
    // IMPORTANT: DO NOT enter on physical B2 with B2 output disabled: legacy
    // native FSM state may stop publishing; this is not a certified safe stop.
    if(!send_b2_){
        const char* ack=std::getenv("B2Z1_REAL_DRYRUN_ACK");
        if(!ack||std::string(ack)!="SIMULATION_ONLY")
            throw std::runtime_error("M2.4e2g no-output mode is SIMULATION_ONLY, not a real B2 state");
    }
    auto b=captureB2();
    float err=0.f,vel=0.f;
    for(int i=0;i<12;++i){err=std::max(err,std::abs(b.motor_q[i]-kStandMotor[i]));
                          vel=std::max(vel,std::abs(b.motor_dq[i]));}
    if(err>.25f||vel>.5f)throw std::runtime_error("B2 not in stable FixStand near expected joint angles");
    const auto c=param::config["interactive_real"];
    std::filesystem::path model=c["policy_path"].as<std::string>();
    if(model.is_relative())model=param::proj_dir/model;
    if(!std::filesystem::is_regular_file(model))throw std::runtime_error("ONNX not found "+model.string());
    runner_=std::make_unique<isaaclab::OrtRunner>(model.string());
    pipeline_=std::make_unique<InteractivePipeline>([this](const Observation799& obs)->Joint18{
        std::unordered_map<std::string,std::vector<float>> inputs;
        inputs.emplace("obs",std::vector<float>(obs.begin(),obs.end()));
        auto raw=runner_->act(std::move(inputs));
        if(raw.size()!=18)throw std::runtime_error("ONNX returned incorrect action count");
        Joint18 a{};std::copy(raw.begin(),raw.end(),a.begin());return a;
    });
    pipeline_->reset();runner_->reset();
    // E2e poller owns the sole Unix IPC connection; it never sends Z1 SDK targets.
    z1_.start(socket_path_,mode_);
    entered_=next_motor_=Clock::now();
    b2_commanded_=b.motor_q;
    b2_desired_=b.motor_q;
    arm_initialized_=false;faulted_=false;walk_enabled_=walk_enabled_&&send_b2_;
    walking_=motion_was_started_=walk_rearm_=false;
    motor_cycles_=policy_steps_=0;output_pending_=false;velocity_={0,0,0};
    if(send_b2_){
        auto& motors=lowcmd->msg_.motor_cmd();
        if(motors.size()<12)throw std::runtime_error("B2 LowCmd<12");
        for(int i=0;i<12;++i){ motors[i].mode(1);motors[i].q(b2_commanded_[i]);
            motors[i].dq(0);motors[i].tau(0);motors[i].kp(b2_kp_);motors[i].kd(b2_kd_);}
    }
    spdlog::info("M2.4e2g ENTER real candidate; mode={} output={} (SDK targets LOCKED)",
                 mode_==m24e2e::Mode::Mock?"MOCK":"SDK",send_b2_);
}

double State_InteractiveArmReal::maxAbs(const Vec6& v){return v.cwiseAbs().maxCoeff();}
void State_InteractiveArmReal::armKey(const std::string& key,const m24e2e::ArmState& current){
    if(!arm_initialized_)return;
    Eigen::Vector3d requested=goal_xyz_;
    if(key=="i")requested.x()+=.01;
    else if(key=="k")requested.x()-=.01;
    else if(key=="j")requested.y()+=.01;
    else if(key=="h")requested.y()-=.01;
    else if(key=="u")requested.z()+=.01;
    else if(key=="n")requested.z()-=.01;
    else return;
    if((requested-origin_xyz_).norm()>.03){spdlog::warn("M2.4e2g IK goal outside 3cm bench envelope");return;}
    HomoMat target=arm_entry_tool_;
    target.block<3,1>(0,3)=requested-mount_-
        arm_entry_tool_.block<3,3>(0,0)*Eigen::Vector3d(tcp_offset_,0.,0.);
    Vec6 q=arm_goal_q_;
    if(!model_.inverseKinematics(target,arm_goal_q_,q,false)||!q.allFinite()){
        spdlog::warn("M2.4e2g IK failed");return;}
    const HomoMat actual=model_.forwardKinematics(q);
    if((actual.block<3,1>(0,3)-target.block<3,1>(0,3)).norm()>.002||
       maxAbs(q-arm_goal_q_)>.65){spdlog::warn("M2.4e2g IK FK consistency rejected");return;}
    const auto qlo=model_.getJointQMin(),qhi=model_.getJointQMax();
    if(qlo.size()!=6||qhi.size()!=6)throw std::runtime_error("Z1 limits unavailable");
    for(int i=0;i<6;++i){
        if(q[i]<qlo[i]+.015||q[i]>qhi[i]-.015||std::abs(q[i]-current.q[i])>.3){
            spdlog::warn("M2.4e2g IK joint limit/current deviation rejected");return;
        }
    }
    arm_goal_q_=q;goal_xyz_=requested;
    spdlog::info("M2.4e2g IK VALID (NOT SENT) XYZ=[{:.3f},{:.3f},{:.3f}]",requested.x(),requested.y(),requested.z());
}
void State_InteractiveArmReal::checkAndUpdateIk(const m24e2e::ArmState& arm){
    Vec6 q=Vec6::Zero();for(int i=0;i<6;++i)q[i]=arm.q[i];
    if(!arm_initialized_){
        arm_goal_q_=q;arm_entry_tool_=model_.forwardKinematics(q);
        const HomoMat tcp=arm_entry_tool_*tcpOffset(tcp_offset_);
        origin_xyz_=goal_xyz_=mount_+tcp.block<3,1>(0,3);
        arm_initialized_=true;
    }
    if(maxAbs(q-arm_goal_q_)>.65)throw std::runtime_error("Z1 feedback vs stored goal deviation >0.65rad");
    if(keyboard && keyboard->on_pressed)armKey(keyboard->key(),arm);
}
void State_InteractiveArmReal::setFault(const std::string& e)noexcept{
    if(!faulted_)spdlog::error("M2.4e2g FAULT: {} (Z1 target always locked)",e);
    faulted_=true;last_error_=e;velocity_={0,0,0};walking_=false;
    output_pending_=false; // Do NOT continue policy output on stale input.
}
void State_InteractiveArmReal::run(){
    if(faulted_)return;
    const auto now=Clock::now();
    if(now<next_motor_)return;
    if(now>next_motor_+kMotorPeriod)next_motor_=now;
    next_motor_+=kMotorPeriod;
    try {
        auto b=captureB2();
        if(now-b.packet_received_at>std::chrono::milliseconds(70))
            throw std::runtime_error("B2 DDS feedback >70ms old");
        m24e2e::ArmState arm;std::string e;
        if(!z1_.latest(arm,e)){
            if(now-entered_<std::chrono::seconds(2))return;
            throw std::runtime_error("Z1 feedback unavailable: "+e);
        }
        if(now-arm.ipc_received_at>std::chrono::milliseconds(80))
            throw std::runtime_error("Z1 IPC response >80ms old (NOT proof of vendor RX freshness)");
        const bool key=keyboard&&keyboard->on_pressed;
        const std::string keyname=keyboard?keyboard->key():"";
        if(key&&keyname==" "){velocity_={0,0,0};walking_=false;walk_rearm_=false;}
        if(walk_enabled_&&key&&keyname=="w"&&!walk_rearm_&&
           now-entered_>=std::chrono::seconds(2)&&policy_steps_>2){
            velocity_={.12f,0,0};walking_=true;walk_rearm_=true;motion_was_started_=true;walk_started_=now;
        }
        if(walking_&&now-walk_started_>std::chrono::seconds(2)){
            walking_=false;velocity_={0,0,0};
        }
        // Do not interpret ONNX arm last-6 outputs as Z1 joint commands.
        if(motor_cycles_%kOnnxDivisor==0){
            checkAndUpdateIk(arm);
            const Command3 ee=use_ik_goal_&&arm_initialized_?
                Command3{float(goal_xyz_.x()),float(goal_xyz_.y()),float(goal_xyz_.z())}:
                Command3{.45f,0,.35f};
            const auto input=m24e2e::makeLiveInput(b,arm,velocity_,ee,now,mode_);
            const auto policy=pipeline_->step(input.robot,input.command);
            const auto legs=makeLegTargets(policy.clipped_action);
            if(legs.saturated>4)throw std::runtime_error("more than 4 leg targets saturated");
            b2_desired_=legs.q_target_motor;
            ++policy_steps_;
            if(policy_steps_%50==1){
                spdlog::info("[M24e2g] 799D/ONNX OK steps={} raw_a12={:.3f} B2out={} Z1target=LOCKED",
                    policy_steps_,policy.raw_action[12],send_b2_);
            }
        }
        if(send_b2_){
            const float step=motion_was_started_?walk_step_:stand_step_;
            b2_commanded_=slewMotorTargets(b2_desired_,b2_commanded_,step);
            auto& motors=lowcmd->msg_.motor_cmd();
            for(int i=0;i<12;++i){
                if(std::abs(b2_commanded_[i]-b.motor_q[i])>1.f)
                    throw std::runtime_error("B2 commanded/measured error exceeds 1rad");
                motors[i].q(b2_commanded_[i]);motors[i].dq(0);motors[i].tau(0);
                motors[i].kp(b2_kp_);motors[i].kd(b2_kd_);
            }
            output_pending_=true;
        }
        ++motor_cycles_;
    } catch(const std::exception& ex){setFault(ex.what());}
}
void State_InteractiveArmReal::post_run(){
    // Only publish 12 B2 leg targets after all checks passed in run().
    if(output_pending_&&!faulted_&&send_b2_){output_pending_=false;FSMState::post_run();}
}
void State_InteractiveArmReal::exit(){
    velocity_={0,0,0};walk_enabled_=false;output_pending_=false;
    z1_.stop();pipeline_.reset();runner_.reset();
    // B2 state transition owns its own command handoff; never send Z1 commands here.
}
