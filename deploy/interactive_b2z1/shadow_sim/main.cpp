// SIMULATION-ONLY, read-only, standalone Interactive shadow inference.
// This binary has no LowCmd or Z1SDK controller and cannot issue motor commands.
// LowState [12..17] is ONLY legal for the joint B2Z1 MuJoCo simulator on loopback.
// Never use it to infer real Z1 feedback, even when real B2 packets have padding.
#include "interactive_feedback_adapter.h"
#include "interactive_pipeline.h"
#include "interactive_snapshot_store.h"
#include "isaaclab/algorithms/algorithms.h"

#include "unitree/dds_wrapper/robots/go2/go2.h"

#include <array>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace interactive_b2z1;
namespace {
using LowState = unitree::robot::go2::subscription::LowState;
std::atomic<bool> running{true};
void stop(int) { running.store(false); }

int parseFrames(const char* value) {
    const auto n = std::stoi(value);
    if (n < 1 || n > 100000) throw std::invalid_argument("frames must be 1..100000");
    return n;
}

void printUsage(const char* app) {
    std::cerr << "Usage (LOOPBACK ONLY): " << app << " <policy.onnx> [frames=250]\n"
              << "No network-interface argument, no motor command publisher, no Z1 SDK.\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc != 2 && argc != 3) { printUsage(argv[0]); return 2; }
        const auto model = std::filesystem::path(argv[1]);
        if (!std::filesystem::is_regular_file(model))
            throw std::runtime_error("ONNX policy missing: " + model.string());
        const int frames = argc == 3 ? parseFrames(argv[2]) : 250;
        std::signal(SIGINT, stop);
        std::signal(SIGTERM, stop);

        isaaclab::OrtRunner runner(model.string());
        InteractivePipeline pipeline([&runner](const Observation799& obs) -> Joint18 {
            std::unordered_map<std::string,std::vector<float>> inputs;
            inputs.emplace("obs",std::vector<float>(obs.begin(),obs.end()));
            const auto raw = runner.act(std::move(inputs));
            if (raw.size() != kDof) throw std::runtime_error("Expected 18 policy outputs");
            Joint18 action{};
            for (std::size_t i=0;i<kDof;++i) action[i]=raw[i];
            return action;
        });

        // Explicitly forced to localhost simulator DDS domain; no CLI escape hatch.
        unitree::robot::ChannelFactory::Instance()->Init(0, "lo");
        auto lowstate = std::make_shared<LowState>();
        std::cerr << "[shadow_sim] Loopback DDS, waiting for simulated LowState ...\n";
        lowstate->wait_for_connection();
        std::cerr << "[shadow_sim] Connected. READ ONLY. No motor commands.\n";

        SnapshotStore snapshots;
        const Command3 stationary_command{0.f,0.f,0.f};
        const Command3 static_ee_target_base{0.45f,0.f,0.35f};
        const auto period = std::chrono::milliseconds(20); // 50 Hz policy
        auto wake = MonoClock::now();
        std::cout << "frame,b2_q0,sim_z1_q0,leg_action0,max_abs_leg_action,step_ms\n";
        for (int frame=0; running.load() && frame<frames; ++frame) {
            const auto started = MonoClock::now();
            // Wrapper freshness check. This polling API does NOT expose an exact
            // receiver callback timestamp, so real-hardware use is prohibited.
            if (lowstate->isTimeout()) throw std::runtime_error("Simulated LowState timeout");
            lowstate->update();
            Joint12 leg_q{}, leg_dq{};
            Arm6 arm_q{}, arm_dq{};
            std::array<float,4> quat{};
            Command3 gyro{};
            {
                std::lock_guard<std::mutex> guard(lowstate->mutex_);
                const auto& msg = lowstate->msg_;
                const auto& motors = msg.motor_state();
                if (motors.size() < 18) throw std::runtime_error("MuJoCo B2Z1 LowState must contain 18 motors");
                for (std::size_t i=0;i<12;++i) {
                    leg_q[i] = motors[i].q();
                    leg_dq[i] = motors[i].dq();
                }
                // IMPORTANT: these six motors exist ONLY in the 18-joint simulator.
                for (std::size_t i=0;i<6;++i) {
                    arm_q[i] = motors[12+i].q();
                    arm_dq[i] = motors[12+i].dq();
                }
                for (std::size_t i=0;i<4;++i) quat[i] = msg.imu_state().quaternion()[i];
                for (std::size_t i=0;i<3;++i) gyro[i] = msg.imu_state().gyroscope()[i];
            }
            if (lowstate->isTimeout()) throw std::runtime_error("Simulated LowState became stale");
            const auto received_at=MonoClock::now();
            snapshots.updateB2(leg_q,leg_dq,quat,gyro,received_at);
            snapshots.updateZ1(arm_q,arm_dq,received_at);
            const auto copy=snapshots.copy(stationary_command,static_ee_target_base);
            const auto input=adaptFeedback(copy,MonoClock::now());
            const auto result=pipeline.step(input.robot,input.command);
            float max_action=0.f;
            for (std::size_t j=0;j<12;++j)
                max_action=std::max(max_action,std::abs(result.clipped_action[j]));
            const auto duration_ms=std::chrono::duration<double,std::milli>(MonoClock::now()-started).count();
            std::cout << frame << ',' << leg_q[0] << ',' << arm_q[0] << ','
                      << result.clipped_action[0] << ',' << max_action << ','
                      << std::fixed << std::setprecision(3) << duration_ms << '\n';
            wake += period;
            std::this_thread::sleep_until(wake);
        }
        std::cerr << "[shadow_sim] completed. No hardware commands were sent.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[shadow_sim] STOPPED (fail closed): " << e.what() << '\n';
        return 1;
    }
}
