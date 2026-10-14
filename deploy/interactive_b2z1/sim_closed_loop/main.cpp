// M1.5 SDK2 COMMUNICATION CLOSED LOOP -- MUJOCO ON 'lo' ONLY.
// --mode inspect: NO LowCmd publisher (default). --mode stand/rl: explicit simulator-only gate.
// NOT for a physical B2, Z1, ROS bridge, or a distributed DDS network.
#include "joint_target_guard.h"
#include "interactive_keyboard_sim.h"
#include "interactive_feedback_adapter.h"
#include "interactive_pipeline.h"
#include "interactive_snapshot_store.h"
#include "isaaclab/algorithms/algorithms.h"
#include "unitree/dds_wrapper/robots/go2/go2.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace interactive_b2z1;
using namespace interactive_b2z1::sim_control;
namespace {
using Clock = MonoClock;
using LowState = unitree::robot::go2::subscription::LowState;
using LowCmdPub = unitree::robot::go2::publisher::LowCmd;
using LowCmdSub = unitree::robot::go2::subscription::LowCmd;
std::atomic<bool> g_run{true};
void onSignal(int) { g_run.store(false); }

struct Cli {
    std::string policy;
    std::string mode{"inspect"};
    int seconds{8};
    bool enable_pub{false};
    float forward_vx{0.f};   // explicitly opt in; stationary by default
    float walk_slew_step{0.008f}; // SIMULATION ONLY: 5ms max target increment during gait
    bool override_walk_slew{false};
    bool v2_reference_command{false}; // SIM ONLY: 0.12 m/s instantaneous command like V2 panel W
    bool hold_walk_slew_after_stop{false}; // SIM ONLY: isolate the command-zero slew transition
    bool interactive_keys{false}; // SIM ONLY: W=0.12, Space=0, Esc=2s stop then release
};
Cli args(int argc, char** argv) {
    if (argc<2) throw std::invalid_argument("Usage: b2z1_sim_closed_loop <policy.onnx> [--mode inspect|stand|rl] [--seconds 8] [--vx 0.15..0.25|--v2-reference-command|--interactive-keys] [--walk-slew-step 0.008..0.08] [--publish-sim-lowcmd]");
    Cli cli; cli.policy=argv[1];
    for(int i=2;i<argc;++i) {
        const std::string s=argv[i];
        if (s=="--mode" && i+1<argc) cli.mode=argv[++i];
        else if(s=="--seconds" && i+1<argc) cli.seconds=std::stoi(argv[++i]);
        else if(s=="--publish-sim-lowcmd") cli.enable_pub=true;
        else if(s=="--v2-reference-command") cli.v2_reference_command=true;
        else if(s=="--hold-walk-slew-after-stop") cli.hold_walk_slew_after_stop=true;
        else if(s=="--interactive-keys") cli.interactive_keys=true;
        else if(s=="--vx" && i+1<argc) cli.forward_vx=std::stof(argv[++i]);
        else if(s=="--walk-slew-step" && i+1<argc) {
            cli.walk_slew_step=std::stof(argv[++i]);
            cli.override_walk_slew=true;
        }
        else throw std::invalid_argument("unknown argument: "+s);
    }
    if(cli.mode!="inspect" && cli.mode!="stand" && cli.mode!="rl")
        throw std::invalid_argument("mode must be inspect, stand, or rl");
    if(cli.seconds<1 || cli.seconds>30) throw std::invalid_argument("seconds must be 1..30");
    if(cli.v2_reference_command) {
        if(cli.mode!="rl" || cli.forward_vx!=0.f)
            throw std::invalid_argument("--v2-reference-command requires --mode rl and no --vx");
        cli.forward_vx=0.12f; // Exactly the V2 panel W command; no speed ramp.
    }
    if(!std::isfinite(cli.forward_vx)) throw std::invalid_argument("invalid forward speed");
    if(cli.interactive_keys) {
        if(cli.mode!="rl" || cli.forward_vx!=0.f || cli.v2_reference_command || cli.hold_walk_slew_after_stop)
            throw std::invalid_argument("--interactive-keys requires --mode rl; not combinable with --vx, --v2-reference-command, or --hold-walk-slew-after-stop");
        if(!cli.override_walk_slew || cli.seconds<12)
            throw std::invalid_argument("--interactive-keys requires --walk-slew-step 0.04 and --seconds >= 12");
        if(std::abs(cli.walk_slew_step-0.04f)>1.e-6f)
            throw std::invalid_argument("M1.6a interactive mode is restricted to validated --walk-slew-step 0.04");
    }
    if(cli.hold_walk_slew_after_stop && (!cli.v2_reference_command || !cli.override_walk_slew))
        throw std::invalid_argument("--hold-walk-slew-after-stop requires --v2-reference-command and --walk-slew-step");
    if(cli.override_walk_slew) {
        if(cli.mode!="rl" || (!cli.interactive_keys && cli.forward_vx==0.f))
            throw std::invalid_argument("--walk-slew-step needs --mode rl and nonzero --vx (or --interactive-keys)");
        if(!std::isfinite(cli.walk_slew_step) || cli.walk_slew_step<0.008f || cli.walk_slew_step>0.08f)
            throw std::invalid_argument("--walk-slew-step must be in [0.008,0.08] rad per 5ms");
    }
    if(cli.forward_vx!=0.f) {
        if(cli.mode!="rl") throw std::invalid_argument("--vx is permitted only with --mode rl");
        if(cli.seconds<10) throw std::invalid_argument("forward test requires --seconds >= 10");
        if(!cli.v2_reference_command && (cli.forward_vx<0.15f || cli.forward_vx>0.25f))
            throw std::invalid_argument("--vx must be in [0.15,0.25] m/s; use 0 to disable");
    }
    if(!std::filesystem::is_regular_file(cli.policy)) throw std::invalid_argument("policy.onnx not found");
    const bool commands=cli.mode!="inspect";
    if(commands) {
        if(!cli.enable_pub) throw std::invalid_argument("refusing actuator publisher without --publish-sim-lowcmd");
        const char* ack=std::getenv("B2Z1_SIM_ONLY_ACK");
        if(!ack || std::string(ack)!="MUJOCO_LOOPBACK_NO_HARDWARE")
            throw std::invalid_argument("refusing actuator publisher: set B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE");
    } else if(cli.enable_pub) throw std::invalid_argument("inspect must not request publisher");
    return cli;
}

struct SimSample {
    Joint12 leg_q{}, leg_dq{}; // simulated motor order FL FR RL RR
    Arm6 arm_q{}, arm_dq{};    // simulated actuator index12..17, NOT real Z1 SDK
    std::array<float,4> quat{};
    Command3 gyro{};
    std::uint32_t sim_tick_ms{0};  // MuJoCo simulated time in milliseconds
};
SimSample readState(LowState& lowstate) {
    SimSample s;
    std::lock_guard<std::mutex> lk(lowstate.mutex_);
    const auto& msg=lowstate.msg_;
    const auto& m=msg.motor_state();
    if(m.size()!=20) throw std::runtime_error("expected SIM LowState with 20 motor slots");
    for(std::size_t i=0;i<12;++i) {
        s.leg_q[i]=m[i].q(); s.leg_dq[i]=m[i].dq();
    }
    for(std::size_t i=0;i<6;++i) {
        s.arm_q[i]=m[12+i].q(); s.arm_dq[i]=m[12+i].dq();
    }
    for(std::size_t i=0;i<4;++i) s.quat[i]=msg.imu_state().quaternion()[i];
    for(std::size_t i=0;i<3;++i) s.gyro[i]=msg.imu_state().gyroscope()[i];
    s.sim_tick_ms=msg.tick();
    checkSimFeedback(s.leg_q,s.leg_dq,s.quat);
    for(const float x:s.arm_q) if(!std::isfinite(x)) throw std::runtime_error("bad Z1 simulated joint state");
    for(const float x:s.arm_dq) if(!std::isfinite(x)) throw std::runtime_error("bad Z1 simulated joint velocity");
    return s;
}

constexpr float kB2Kp=360.f;  // Python V2 MuJoCo baseline ONLY (NOT hardware gains)
constexpr float kB2Kd=5.f;
constexpr float kZ1Kp=90.f;
constexpr float kZ1Kd=6.f;
constexpr float kSlewStep=0.008f;  // max target difference per 5ms tick, sim only
constexpr double kStandRampSeconds=2.5;
constexpr std::chrono::milliseconds kTick{5}; // 200Hz PD; RL/obs 50Hz
constexpr Command3 kZeroCmd{0.f,0.f,0.f};
constexpr Command3 kGoalBase{0.45f,0.f,0.35f};
// Conservative open-loop COMMAND schedule (policy computes closed-loop joint targets):
// 0..2.5 s stand; 2.5..4.5 s stationary RL;
// 4.5..5.25 s vx ramps up; vx holds until (seconds-2);
// next 0.75 s ramps down; at least 1.25 s stationary RL before shutdown.
// The command is sampled at 50Hz and enters the verified 799D observation path.
Command3 velocityCommandAt(const Cli& cli, double t) {
    if(cli.mode!="rl" || cli.forward_vx==0.f) return kZeroCmd;
    // Match the previously successful Python V2 panel W profile: 0 -> 0.12
    // immediately, with no transitional commands in (0, 0.1). Keep startup,
    // RL stand, shutdown, joint-limit and torso-tilt protections unchanged.
    if(cli.v2_reference_command) {
        return (t>=4.5 && t<static_cast<double>(cli.seconds)-2.0)
            ? Command3{0.12f,0.f,0.f} : kZeroCmd;
    }
    constexpr double start=4.5;
    constexpr double ramp=0.75;
    const double end=static_cast<double>(cli.seconds)-2.0;
    const double ramp_up=std::clamp((t-start)/ramp,0.0,1.0);
    const double ramp_down=std::clamp((end+ramp-t)/ramp,0.0,1.0);
    const double factor=std::min(ramp_up,ramp_down);
    return {static_cast<float>(cli.forward_vx*factor),0.f,0.f};
}
const Joint12 kDefaultMotor = [] {
    Joint18 zeros{};
    return makeLegTargets(zeros).q_target_motor;
}();

void sendCommand(LowCmdPub& pub, const Joint12& leg, const Arm6& arm) {
    pub.lock();
    auto& m=pub.msg_.motor_cmd();
    if (m.size()<20) { pub.unlock(); throw std::runtime_error("LowCmd motor array < 20"); }
    for(std::size_t i=0;i<20;++i) {
        m[i].mode(1);
        m[i].q(0.f); m[i].dq(0.f); m[i].kp(0.f);m[i].kd(0.f);m[i].tau(0.f);
    }
    for(std::size_t i=0;i<12;++i) {
        m[i].q(leg[i]);m[i].kp(kB2Kp);m[i].kd(kB2Kd);
    }
    for(std::size_t i=0;i<6;++i) {
        m[12+i].q(arm[i]);m[12+i].kp(kZ1Kp);m[12+i].kd(kZ1Kd);
    }
    pub.unlockAndPublish();
}

// SIMULATION ONLY! Remove active position gains and send only damping for 100ms.
// The MuJoCo robot MAY FALL. Never call this a physical robot's safe stop.
void releaseSimCommands(LowCmdPub& pub) noexcept {
    try {
        for(int tick=0;tick<20;++tick) {
            pub.lock();
            auto& m=pub.msg_.motor_cmd();
            for(std::size_t i=0;i<m.size();++i) {
                m[i].mode(1);m[i].q(0.f);m[i].dq(0.f);
                m[i].kp(0.f);m[i].kd(i<18?2.f:0.f);m[i].tau(0.f);
            }
            pub.unlockAndPublish();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    } catch (...) { std::cerr << "[sim] WARNING: unable to send final damping messages\n"; }
}

void assertNoOtherPublisher() {
    auto other=std::make_shared<LowCmdSub>();
    // LowCmd from another controller MUST NOT coexist with this simulator test.
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    if(!other->isTimeout()) throw std::runtime_error("another LowCmd publisher is active; abort test");
    other.reset();
}

void run(const Cli& cli) {
    // Keyboard input only controls the locally simulated speed command;
    // it neither sends DDS messages nor initializes a hardware transport.
    InteractiveTerminalInput terminal(cli.interactive_keys);
    InteractiveKeyboardState keyboard_state;
    isaaclab::OrtRunner runner(cli.policy);
    InteractivePipeline pipeline([&runner](const Observation799& obs) -> Joint18 {
        std::unordered_map<std::string,std::vector<float>> inputs;
        inputs.emplace("obs",std::vector<float>(obs.begin(),obs.end()));
        const auto raw=runner.act(std::move(inputs));
        if(raw.size()!=kDof) throw std::runtime_error("policy must output 18 floats");
        Joint18 v{};
        for(std::size_t i=0;i<kDof;++i) v[i]=raw[i];
        return v;
    });
    // Hard-coded interface and DDS domain match the currently validated local MuJoCo.
    // Domain/network arguments intentionally not exposed.
    unitree::robot::ChannelFactory::Instance()->Init(0,"lo");
    auto lowstate=std::make_shared<LowState>();
    lowstate->set_timeout_ms(120);
    std::cerr << "[sim] Waiting for MuJoCo LowState on lo (domain 0)\n";
    lowstate->wait_for_connection();
    if(lowstate->isTimeout()) throw std::runtime_error("DDS disconnected at initialization");
    lowstate->update();
    const SimSample initial=readState(*lowstate);
    // A 20-element SDK2 motor_state is a fixed-size message, NOT proof of
    // simulator model identity. Likewise, zero-valued Z1 joints are valid:
    // the MuJoCo home keyframe is not necessarily activated at startup.
    // READ-ONLY inspect must report actual measurements instead of guessing
    // model identity from the initial arm pose.
    std::cerr << "[sim] initial B2 motor q=[";
    for(std::size_t i=0; i<initial.leg_q.size(); ++i)
        std::cerr << (i ? "," : "") << initial.leg_q[i];
    std::cerr << "] Z1 joint q=[";
    for(std::size_t i=0; i<initial.arm_q.size(); ++i)
        std::cerr << (i ? "," : "") << initial.arm_q[i];
    std::cerr << "] mode=" << cli.mode << "\n";

    // A near-zero Z1 joint angle is legal and not evidence of a wrong model.
    // The MODEL identity must be verified separately against the simulator's
    // printed 18-actuator list; the fixed-size 20-slot DDS message is not proof.
    // For ACTUATING modes, require a B2 pose close to the known nominal stand
    // and low joint velocities. This rejects a fallen, though level, robot.
    // The current scene's home keyframe has rear thigh=1.0 rad, whereas the
    // policy's nominal posture has rear thigh=0.8 rad: 0.2 rad is expected.
    float max_stand_error = 0.f;
    float max_abs_leg_speed = 0.f;
    std::size_t worst_joint = 0;
    for(std::size_t i=0;i<12;++i) {
        const float err = std::abs(initial.leg_q[i]-kDefaultMotor[i]);
        if(err>max_stand_error) {
            max_stand_error=err;
            worst_joint=i;
        }
        max_abs_leg_speed=std::max(max_abs_leg_speed,std::abs(initial.leg_dq[i]));
    }
    std::cerr << "[sim] B2 start preflight max_stand_error=" << max_stand_error
              << " rad, motor_index=" << worst_joint
              << ", max_abs_leg_speed=" << max_abs_leg_speed << " rad/s\n";
    if(cli.mode!="inspect") {
        // Conservative simulation-start readiness test, NOT a hardware guard.
        if(max_stand_error>0.35f || max_abs_leg_speed>0.50f)
            throw std::runtime_error(
                "SIM start not near home/stand pose or legs still moving. "
                "Pause MuJoCo, select keyframe 'home' and Load key. "
                "Do not attempt stand/rl from a fallen configuration.");
        std::cerr << "[sim] B2 standing-pose preflight PASS. "
                     "Operator must also verify the 18-actuator B2Z1 scene, "
                     "loopback isolation, and no other LowCmd publisher.\n";
    }

    std::unique_ptr<LowCmdPub> pub;
    if(cli.mode!="inspect") {
        assertNoOtherPublisher();
        if(lowstate->isTimeout()) throw std::runtime_error("LowState stale before publisher initialization");
        pub=std::make_unique<LowCmdPub>();
        std::cerr << "[sim] SIMULATION LOWCMD ACTIVE; press Ctrl+C to stop (robot may fall)\n";
    } else {
        std::cerr << "[sim] READ-ONLY inspect: no LowCmd publisher instantiated\n";
    }

    SnapshotStore snapshots;
    Joint12 commanded=initial.leg_q;
    Joint12 desired=kDefaultMotor;
    const Arm6 arm_hold=initial.arm_q;
    auto next=Clock::now();
    // The simulator is initially paused at its home keyframe. Use the SDK2
    // bridge's simulation-time tick (not controller wall time) for state stages.
    // This prevents an operator's delay in pressing Space from consuming the
    // 2.5 s stand preparation. The publisher holds the initial pose while paused.
    const std::uint32_t initial_sim_tick_ms=initial.sim_tick_ms;
    std::uint32_t last_sim_tick_ms=initial_sim_tick_ms;
    const auto controller_start=Clock::now();
    auto last_sim_progress=controller_start;
    bool simulation_has_advanced=false;
    bool rl_active_announced=false;
    bool walking_announced=false;
    bool post_stop_slew_announced=false;
    bool keyboard_exit_announced=false;
    bool keyboard_auto_stop_announced=false;
    if(cli.interactive_keys) {
        std::cerr << "[keys] terminal W=forward 0.12 (max 2s per W), SPACE=stop/rearm, Esc=2s stop then exit. "
                     "Press keys in THIS terminal, not the MuJoCo viewer. "
                     "Forward enabled after sim t=4.5s and disabled in final 2s.\n";
        std::cerr << "[keys] SIM ONLY: after normal exit LowCmd gains are released and model may fall.\n";
    }
    // Diagnostic-only: records whether the 1.6 rad/s target slew limiter
    // delays RL joint targets when the policy starts gait motion.
    // Does not alter actions, PD gains, limits, or the safety checks.
    // 50Hz diagnostics: target error covers all 12 legs, not just motor[0].
    // "motor_index" follows THIS MuJoCo bridge's FL/FR/RL/RR indexing.
    std::cout << "time_s,mode,policy_active,cmd_vx,b2_q0,b2_q0_target,z1_q2,"
                 "leg_action0,max_abs_leg_action,saturated,"
                 "max_abs_b2_tracking_error,worst_motor_index,max_abs_b2_joint_speed,"
                 "max_abs_z1_hold_error,roll_rad,pitch_rad,compute_ms,"
                 "gait_phase,slew_limited_joints,max_abs_target_lag,"
                 "worst_lag_motor_index,gyro_pitch_rad_s,active_slew_step\n";

    try {
        // Absolute wall-time bound also covers the operator never starting
        // physics. Duration requested by --seconds measures simulation time.
        const int wall_tick_limit=(cli.seconds+20)*200;
        for(int tick=0;g_run.load() && tick<wall_tick_limit;++tick) {
            const auto started=Clock::now();
            if(lowstate->isTimeout()) throw std::runtime_error("MuJoCo LowState timeout");
            lowstate->update();
            const SimSample state=readState(*lowstate);
            if(lowstate->isTimeout()) throw std::runtime_error("MuJoCo LowState timeout after read");
            if(pub) {
                const auto now=Clock::now();
                if(state.sim_tick_ms<last_sim_tick_ms)
                    throw std::runtime_error("MuJoCo simulation time moved backwards: unexpected reset; abort controller");
                if(state.sim_tick_ms>last_sim_tick_ms) {
                    last_sim_progress=now;
                    if(!simulation_has_advanced) {
                        simulation_has_advanced=true;
                        std::cerr << "[sim] MUJOCO PHYSICS ADVANCING; starting 2.5 s stand ramp in simulation time\n";
                    }
                }
                if(!simulation_has_advanced && now-controller_start>std::chrono::seconds(12))
                    throw std::runtime_error("MuJoCo remained paused for 12 seconds: abort test (no auto-unpause)");
                if(simulation_has_advanced && now-last_sim_progress>std::chrono::milliseconds(700))
                    throw std::runtime_error("MuJoCo simulation clock stopped for >700ms; abort instead of advancing RL phases");
                last_sim_tick_ms=state.sim_tick_ms;
            }
            if(pub && (std::abs(state.arm_q[1]-arm_hold[1])>0.6f ||
                       std::abs(state.arm_q[2]-arm_hold[2])>0.6f))
                throw std::runtime_error("Z1 arm joint deviated from locked SIM pose");

            // INSPECT retains wall-clock frame time. Actuating modes use the
            // clock embedded in the simulator's LowState; manual Space timing
            // no longer changes where Stand -> RL -> forward transitions occur.
            const double t=pub
                ? (static_cast<double>(state.sim_tick_ms-initial_sim_tick_ms)*0.001)
                : (static_cast<double>(tick)*0.005);
            if(pub && simulation_has_advanced && t>=cli.seconds) break;
            if(cli.interactive_keys) {
                // Nonblocking read every 5ms, independent of 50Hz policy cadence.
                terminal.poll([&](char key) {
                    const auto event=keyboard_state.feed(key,t,cli.seconds);
                    if(event==KeyboardEvent::Forward)
                        std::cerr << "[keys] W -> vx=0.12 at sim t=" << t << "s\n";
                    else if(event==KeyboardEvent::Stop)
                        std::cerr << "[keys] SPACE -> vx=0 (keep walking slew for recovery) at sim t=" << t << "s\n";
                    else if(event==KeyboardEvent::Exit)
                        std::cerr << "[keys] Esc -> vx=0, allow 2s stop recovery then release SIM LowCmd at sim t=" << t << "s\n";
                    else if(event==KeyboardEvent::TooEarly)
                        std::cerr << "[keys] W ignored until RL settles (sim t>=4.5s)\n";
                });
                if(keyboard_state.forwardTimedOut(t) && !keyboard_auto_stop_announced) {
                    keyboard_auto_stop_announced=true;
                    std::cerr << "[keys] 2s forward limit -> vx=0; press SPACE to rearm W\n";
                }
                if(keyboard_state.readyToExit(t) && !keyboard_exit_announced) {
                    keyboard_exit_announced=true;
                    std::cerr << "[keys] 2s stop recovery elapsed; ending SIM control (robot may fall)\n";
                    break;
                }
            }
            const Command3 user_command = cli.interactive_keys
                ? Command3{keyboard_state.velocity(t,cli.seconds),0.f,0.f}
                : velocityCommandAt(cli,t);
            // Isolate the rate-limit comparison to the walking portion ONLY.
            // The stand ramp and stationary RL continue with the previously
            // validated 0.008 rad/5ms limiter, regardless of the CLI override.
            const bool walking_now = cli.mode=="rl" && user_command[0] > 0.1f;
            // Diagnostic AB: after command vx returns to zero, the old controller
            // instantly dropped slew from 0.04 back to 0.008 rad/5ms. This is
            // during gait-phase reset and while the body is still moving.
            // Keep the walking slew for the final stop/recovery interval ONLY
            // when explicitly opted in; never change tilt/joint/timeout guards.
            const bool stop_recovery = (cli.hold_walk_slew_after_stop &&
                t>=static_cast<double>(cli.seconds)-2.0 && t<cli.seconds) ||
                (cli.interactive_keys && keyboard_state.everMoved() && t>=4.5);
            // In keyboard mode once motion starts, retain 0.04 through every
            // STOP command. Reverting to 0.008 caused the M1.5m stop failure.
            const float active_slew_step = (walking_now || stop_recovery)
                ? cli.walk_slew_step : kSlewStep;
            if(pub && stop_recovery && !walking_now && !post_stop_slew_announced) {
                std::cerr << "[sim] STOP RECOVERY SLEW HELD: cmd_vx=0, step="
                          << active_slew_step << " rad/5ms at t=" << t << " s\n";
                post_stop_slew_announced=true;
            }
            float a0=0.f, maxa=0.f;
            std::size_t saturated=0;
            if((tick%4)==0) {
                const auto ts=Clock::now();
                snapshots.updateB2(state.leg_q,state.leg_dq,state.quat,state.gyro,ts);
                snapshots.updateZ1(state.arm_q,state.arm_dq,ts); // SIM joint motors ONLY
                const auto input=adaptFeedback(snapshots.copy(user_command,kGoalBase),Clock::now());
                if(pub && !walking_announced && user_command[0]>0.1f) {
                    std::cerr << "[sim] FORWARD COMMAND ACTIVE: vx=" << user_command[0]
                              << " m/s at t=" << t << " s (requested=" << (cli.interactive_keys ? 0.12f : cli.forward_vx)
                              << ", walking target slew=" << active_slew_step
                              << " rad/5ms)\n";
                    walking_announced=true;
                }
                if(cli.mode=="inspect" || (cli.mode=="rl" && t>=kStandRampSeconds)) {
                    const auto result=pipeline.step(input.robot,input.command);
                    const auto targets=makeLegTargets(result.clipped_action);
                    if(cli.mode=="rl") {
                        desired=targets.q_target_motor;
                        if(!rl_active_announced) {
                            std::cerr << "[sim] RL POLICY ACTIVE at t=" << t
                                      << " s; B2=Interactive 12-DOF, Z1=hold, commanded vx="
                                      << (cli.interactive_keys ? 0.12f : cli.forward_vx)
                                      << (cli.interactive_keys ? " (W/Space SIM keyboard)\n" : " (staged speed test if nonzero)\n");
                            rl_active_announced=true;
                        }
                    }
                    saturated=targets.saturated;
                    a0=result.clipped_action[0];
                    for(std::size_t j=0;j<12;++j) maxa=std::max(maxa,std::abs(result.clipped_action[j]));
                    if(saturated>4 && pub) {
                        // SIM ONLY: record the *offending* inference before the
                        // existing guard aborts.  The ordinary 50Hz CSV is not
                        // printed for this frame, which previously hid the cause.
                        std::cerr << std::fixed << std::setprecision(4)
                                  << "[sim] SATURATION_DIAG t=" << t
                                  << " tick_ms=" << state.sim_tick_ms
                                  << " vx=" << user_command[0]
                                  << " phase=" << result.observation[66]
                                  << " roll=" << result.observation[0]
                                  << " pitch=" << result.observation[1]
                                  << " saturated=" << saturated << '\n';
                        std::cerr << "[sim] SATURATION_DIAG arm_q=[";
                        for(std::size_t k=0;k<6;++k)
                            std::cerr << (k ? "," : "") << state.arm_q[k];
                        std::cerr << "] ee_obs=["
                                  << result.observation[60] << ','
                                  << result.observation[61] << ','
                                  << result.observation[62] << "]\n";
                        constexpr std::array<float,3> kLegScale{0.4f,0.45f,0.45f};
                        for(std::size_t j=0;j<12;++j) {
                            const std::size_t motor=kPolicyToB2Motor[j];
                            const float raw_target=kDefaultMotor[motor]
                                + kLegScale[j%3]*result.clipped_action[j];
                            std::cerr << "[sim] SATURATION_DIAG policy_joint=" << j
                                      << " motor=" << motor
                                      << " raw_action=" << result.clipped_action[j]
                                      << " raw_target=" << raw_target
                                      << " guarded=" << targets.q_target_policy[j]
                                      << " actual_q=" << state.leg_q[motor]
                                      << " actual_dq=" << state.leg_dq[motor]
                                      << " commanded_prev=" << commanded[motor]
                                      << '\n';
                        }
                        throw std::runtime_error("excessive joint target saturation; policy out of domain");
                    }
                }
                if(pub && (cli.mode=="stand" || t<kStandRampSeconds)) {
                    // Reconcile observed initial posture with nominal posture smoothly.
                    const float alpha=static_cast<float>(std::clamp(t/kStandRampSeconds,0.0,1.0));
                    for(std::size_t j=0;j<12;++j)
                        desired[j]=initial.leg_q[j]+alpha*(kDefaultMotor[j]-initial.leg_q[j]);
                }
                float max_tracking_error=0.f;
                float max_leg_speed=0.f;
                float max_arm_hold_error=0.f;
                std::size_t worst_motor=0;
                for(std::size_t j=0;j<12;++j) {
                    const float err=std::abs(state.leg_q[j]-commanded[j]);
                    if(err>max_tracking_error) {
                        max_tracking_error=err;
                        worst_motor=j;
                    }
                    max_leg_speed=std::max(max_leg_speed,std::abs(state.leg_dq[j]));
                }
                for(std::size_t j=0;j<6;++j)
                    max_arm_hold_error=std::max(max_arm_hold_error,
                                                std::abs(state.arm_q[j]-arm_hold[j]));
                const auto& q=state.quat;
                const double quat_norm=std::sqrt(static_cast<double>(q[0])*q[0] +
                    static_cast<double>(q[1])*q[1] + static_cast<double>(q[2])*q[2] +
                    static_cast<double>(q[3])*q[3]);
                const double w=q[0]/quat_norm, x=q[1]/quat_norm,
                             y=q[2]/quat_norm, z=q[3]/quat_norm;
                const double roll=std::atan2(2.*(w*x+y*z),1.-2.*(x*x+y*y));
                const double pitch=std::asin(std::clamp(2.*(w*y-z*x),-1.,1.));
                // Desired vs. last *actually published* B2 joint target, in
                // simulator FL/FR/RL/RR motor order. If limited_joints stays
                // nonzero while tracking_error grows, investigate target
                // rate limiting before changing the learned policy.
                float max_target_lag=0.f;
                std::size_t worst_lag_motor=0;
                int slew_limited_joints=0;
                for(std::size_t j=0;j<12;++j) {
                    const float lag=std::abs(desired[j]-commanded[j]);
                    if(lag>max_target_lag) {
                        max_target_lag=lag;
                        worst_lag_motor=j;
                    }
                    if(lag>active_slew_step+1.e-5f) ++slew_limited_joints;
                }
                // Proprio index 66 is precisely the phase passed to ONNX.
                const float gait_phase=pipeline.observationBuilder().lastProprio()[66];
                const double compute_ms=std::chrono::duration<double,std::milli>(Clock::now()-started).count();
                const Command3 current_command=user_command;
                std::cout << std::fixed << std::setprecision(4) << t << ',' << cli.mode << ','
                          << (cli.mode=="rl" && t>=kStandRampSeconds ? 1 : 0) << ','
                          << current_command[0] << ','
                          << state.leg_q[0] << ',' << commanded[0] << ',' << state.arm_q[1] << ','
                          << a0 << ',' << maxa << ',' << saturated << ','
                          << max_tracking_error << ',' << worst_motor << ',' << max_leg_speed << ','
                          << max_arm_hold_error << ',' << roll << ',' << pitch << ',' << compute_ms << ','
                          << gait_phase << ',' << slew_limited_joints << ',' << max_target_lag << ','
                          << worst_lag_motor << ',' << state.gyro[1] << ','
                          << active_slew_step << '\n';
            }
            if(pub) {
                commanded=slewMotorTargets(desired,commanded,active_slew_step);
                for(std::size_t i=0;i<12;++i)
                    if(std::abs(commanded[i]-state.leg_q[i])>1.0f)
                        throw std::runtime_error("large joint tracking error; stop test");
                sendCommand(*pub,commanded,arm_hold);
            }
            next+=kTick;
            // A missed schedule deadline by >100ms is a hard error while commanding.
            if(pub && Clock::now()>next+std::chrono::milliseconds(100))
                throw std::runtime_error("200Hz simulation command loop overrun >100ms");
            std::this_thread::sleep_until(next);
        }
        if(pub && !simulation_has_advanced)
            throw std::runtime_error("MuJoCo physics never advanced; cannot validate stand or RL");
        if(pub && Clock::now()-controller_start > std::chrono::seconds(cli.seconds+19))
            throw std::runtime_error("simulation did not complete before wall-clock watchdog");
        std::cerr << "[sim] completed mode="<<cli.mode<<"; releasing SIM LowCmd if active\n";
    } catch (...) {
        if(pub) releaseSimCommands(*pub);
        throw;
    }
    if(pub) releaseSimCommands(*pub);
}
} // namespace

int main(int argc,char** argv) {
    try {
        const Cli cli=args(argc,argv);
        std::signal(SIGINT,onSignal);
        std::signal(SIGTERM,onSignal);
        run(cli);
        return 0;
    } catch(const std::exception& e) {
        std::cerr << "[sim] STOP: "<<e.what()<<'\n';
        return 1;
    }
}
