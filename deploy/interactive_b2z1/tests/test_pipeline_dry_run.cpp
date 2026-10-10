// M1.4a offline closed-loop pipeline test; NEVER sends robot commands.
// Usage: test_pipeline_dry_run <b2z1_interactive_policy.onnx> [frames=25]
// Output CSV rows: frame,799 observation values,18 clipped policy actions.

#include "interactive_pipeline.h"
#include "isaaclab/algorithms/algorithms.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

using namespace interactive_b2z1;
namespace {
constexpr std::array<Command3, 5> kCommands{{
    {0.f, 0.f, 0.f}, {0.2f, 0.f, 0.f}, {0.25f, -0.05f, 0.1f},
    {0.f, 0.f, 0.f}, {-0.2f, 0.f, 0.f}
}};

RobotSample sample(int frame, const ObservationConfig& cfg) {
    const int f = frame % 5; // Repeated deterministic state samples, bounded IMU norm.
    RobotSample s;
    Joint12 motor_q{};
    Joint12 motor_dq{};
    for (std::size_t i = 0; i < kLegDof; ++i) {
        const std::size_t motor_id = kPolicyToB2Motor[i];
        motor_q[motor_id] = cfg.default_joint_pos[i] +
            0.003f * static_cast<float>((i + 1) * (f + 1));
        motor_dq[motor_id] = -0.02f * static_cast<float>((i + 1) * (f + 1));
    }
    const Joint12 policy_q = b2MotorToPolicy(motor_q);
    const Joint12 policy_dq = b2MotorToPolicy(motor_dq);
    for (std::size_t i = 0; i < kLegDof; ++i) {
        s.joint_q[i] = policy_q[i];
        s.joint_dq[i] = policy_dq[i];
    }
    for (std::size_t i = kLegDof; i < kDof; ++i) {
        s.joint_q[i] = cfg.default_joint_pos[i] +
            0.003f * static_cast<float>((i + 1) * (f + 1));
        s.joint_dq[i] = -0.02f * static_cast<float>((i + 1) * (f + 1));
    }
    s.imu_quat_wxyz = {1.f + 0.001f * f, 0.018f * (f+1),
                       -0.012f * (f+1), 0.015f};
    s.imu_gyro_xyz = {0.1f * (f+1), -0.2f * (f+1), 0.03f * (f+1)};
    s.ee_local_xyz = {0.34f - 0.01f * f, -0.06f + 0.005f * f,
                      0.12f + 0.015f * f};
    return s;
}
}

int main(int argc, char** argv) {
    try {
        if (argc < 2 || argc > 3) {
            std::cerr << "Usage: test_pipeline_dry_run policy.onnx [frames=25]\n";
            return 2;
        }
        const int nframes = argc == 3 ? std::stoi(argv[2]) : 25;
        if (nframes < 2 || nframes > 1000) throw std::invalid_argument("frames must be 2..1000");
        // Use the actual preexisting deployment OrtRunner; no alternate inference implementation.
        isaaclab::OrtRunner runner(argv[1]);
        InteractivePipeline pipeline([&runner](const Observation799& obs) {
            std::unordered_map<std::string, std::vector<float>> inputs;
            inputs.emplace("obs", std::vector<float>(obs.begin(), obs.end()));
            const auto action = runner.act(std::move(inputs));
            if (action.size() != kDof) throw std::runtime_error("Policy output must have 18 values");
            Joint18 raw{};
            for (std::size_t i = 0; i < kDof; ++i) raw[i] = action[i];
            return raw;
        });
        const auto cfg = ObservationConfig::baseline();
        std::cout << std::setprecision(9);
        for (int i = 0; i < nframes; ++i) {
            const auto result = pipeline.step(sample(i, cfg), kCommands[i % 5]);
            std::cout << i;
            for (float value : result.observation) std::cout << ',' << value;
            for (float value : result.clipped_action) std::cout << ',' << value;
            std::cout << '\n';
        }
        return 0;
    } catch (const Ort::Exception& e) {
        std::cerr << "ONNX error: " << e.what() << '\n';
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "M1.4a error: " << e.what() << '\n';
        return 1;
    }
}
