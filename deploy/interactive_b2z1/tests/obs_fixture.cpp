#include "interactive_observation.h"

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace interactive_b2z1;

namespace {
constexpr int kFrames = 5;
const std::array<Command3, kFrames> kCommands{{
    {0.f, 0.f, 0.f},
    {0.2f, 0.f, 0.f},
    {0.25f, -0.05f, 0.1f},
    {0.f, 0.f, 0.f},
    {-0.2f, 0.f, 0.f},
}};

RobotSample makeSample(int frame, const ObservationConfig& cfg) {
    RobotSample s;
    // Intentionally construct motor order first, then apply the confirmed mapping.
    Joint12 motor_q{};
    Joint12 motor_dq{};
    for (std::size_t i = 0; i < kLegDof; ++i) {
        const std::size_t motor_id = kPolicyToB2Motor[i];
        motor_q[motor_id] = cfg.default_joint_pos[i] +
            0.003f * static_cast<float>((i+1)*(frame+1));
        motor_dq[motor_id] = -0.02f * static_cast<float>((i+1)*(frame+1));
    }
    const auto policy_q = b2MotorToPolicy(motor_q);
    const auto policy_dq = b2MotorToPolicy(motor_dq);
    for (std::size_t i = 0; i < kLegDof; ++i) {
        s.joint_q[i] = policy_q[i];
        s.joint_dq[i] = policy_dq[i];
    }
    for (std::size_t i = 12; i < kDof; ++i) {
        s.joint_q[i] = cfg.default_joint_pos[i] +
            0.003f * static_cast<float>((i+1)*(frame+1));
        s.joint_dq[i] = -0.02f * static_cast<float>((i+1)*(frame+1));
    }
    s.imu_quat_wxyz = {1.f + 0.001f*frame, 0.018f*(frame+1),
                        -0.012f*(frame+1), 0.015f};
    s.imu_gyro_xyz = {0.1f*(frame+1), -0.2f*(frame+1), 0.03f*(frame+1)};
    s.ee_local_xyz = {0.34f-0.01f*frame, -0.06f+0.005f*frame,
                      0.12f+0.015f*frame};
    return s;
}

Joint18 makeAction(int frame) {
    Joint18 action{};
    for (std::size_t i = 0; i < kDof; ++i) {
        action[i] = 0.1f * static_cast<float>((i+1)*(frame+1));
    }
    if (frame == 2) action[0] = 125.f; // Test Python clip(raw, -100, 100)
    return action;
}

void check(bool predicate, const std::string& msg) {
    if (!predicate) throw std::runtime_error(msg);
}

void selftest() {
    static_assert(kObsDim == 799, "Unexpected observation layout");
    const auto cfg = ObservationConfig::baseline();
    ObservationBuilder builder(cfg);
    const RobotSample s0 = makeSample(0, cfg);
    const Observation799 o0 = builder.build(s0, kCommands[0]);
    check(builder.stepCount() == 1, "step counter not updated");
    check(o0[71+5] == 1.f, "privileged placeholder offset wrong");
    check(o0[66] == 0.f, "gait phase should reset on zero command");
    for (std::size_t i = 89; i < 799; ++i) {
        check(o0[i] == 0.f, "initial history must be all zeros");
    }
    const auto first_p = builder.lastProprio();
    builder.acceptPolicyAction(makeAction(0));
    const Observation799 o1 = builder.build(makeSample(1, cfg), kCommands[1]);
    check(builder.stepCount() == 2, "second step counter mismatch");
    for (std::size_t h = 0; h < 10; ++h) {
        for (std::size_t i = 0; i < 71; ++i) {
            check(o1[89 + h*71 + i] == first_p[i], "history pre-advance mismatch");
        }
    }
    check(std::abs(o1[41] - 0.1f) < 1e-6f, "last action lag incorrect");
    const auto old_step = builder.stepCount();
    auto invalid = makeSample(2, cfg);
    invalid.joint_q[3] = std::numeric_limits<float>::quiet_NaN();
    bool rejected = false;
    try { (void)builder.build(invalid, kCommands[2]); }
    catch (const std::invalid_argument&) { rejected = true; }
    check(rejected && builder.stepCount() == old_step, "rejected input must not advance history");
    builder.reset();
    check(builder.stepCount() == 0 && builder.gaitPhase() == 0, "reset incomplete");
    check(builder.build(s0, kCommands[0]) == o0, "reset first observation differs");
    std::cout << "SELFTEST PASS\n";
}
} // namespace

int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--selftest") {
            selftest();
            return 0;
        }
        const auto cfg = ObservationConfig::baseline();
        if (argc == 2 && std::string(argv[1]) == "--config") {
            std::cout << std::setprecision(9);
            for (float f : cfg.default_joint_pos) std::cout << f << ',';
            for (float f : cfg.cmd_scale) std::cout << f << ',';
            std::cout << cfg.control_dt << ',' << cfg.gait_frequency_hz << ','
                      << cfg.dof_pos_scale << ',' << cfg.dof_vel_scale << ','
                      << cfg.ang_vel_scale << ',' << cfg.max_abs_action << '\n';
            return 0;
        }
        ObservationBuilder builder(cfg);
        std::cout << std::setprecision(9);
        for (int frame = 0; frame < kFrames; ++frame) {
            const Observation799 obs = builder.build(makeSample(frame, cfg), kCommands[frame]);
            std::cout << frame;
            for (float f : obs) std::cout << ',' << f;
            std::cout << '\n';
            builder.acceptPolicyAction(makeAction(frame));
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR " << e.what() << '\n';
        return 1;
    }
}
