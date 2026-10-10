#include "interactive_observation.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>

namespace interactive_b2z1 {
namespace {
constexpr double kPi = 3.14159265358979323846264338327950288;

template <std::size_t N>
void requireFinite(const std::array<float, N>& values, const char* field) {
    for (const float value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(std::string("non-finite ") + field);
        }
    }
}

void validateConfig(const ObservationConfig& cfg) {
    requireFinite(cfg.default_joint_pos, "default_joint_pos");
    requireFinite(cfg.cmd_scale, "cmd_scale");
    const std::array<float, 6> scalars = {
        cfg.control_dt, cfg.gait_frequency_hz, cfg.dof_pos_scale,
        cfg.dof_vel_scale, cfg.ang_vel_scale, cfg.max_abs_action
    };
    requireFinite(scalars, "config scalars");
    if (cfg.control_dt <= 0.f || cfg.gait_frequency_hz <= 0.f ||
        cfg.max_abs_action <= 0.f) {
        throw std::invalid_argument("invalid control_dt / frequency / max_abs_action");
    }
}
} // namespace

Joint12 b2MotorToPolicy(const Joint12& motor_order) {
    requireFinite(motor_order, "B2 motor state");
    Joint12 ordered{};
    for (std::size_t i = 0; i < kLegDof; ++i) {
        ordered[i] = motor_order[kPolicyToB2Motor[i]];
    }
    return ordered;
}

ObservationConfig ObservationConfig::baseline() {
    ObservationConfig cfg;
    cfg.default_joint_pos = {
        -0.1f, 0.8f, -1.5f, 0.1f, 0.8f, -1.5f,
        -0.1f, 0.8f, -1.5f, 0.1f, 0.8f, -1.5f,
        0.f, 1.48f, -0.63f, -0.84f, 0.f, 1.57f
    };
    return cfg;
}

ObservationBuilder::ObservationBuilder(ObservationConfig config) : cfg_(std::move(config)) {
    validateConfig(cfg_);
    reset();
}

void ObservationBuilder::reset() {
    gait_phase_ = 0.0;
    step_count_ = 0;
    last_action_.fill(0.f);
    last_proprio_.fill(0.f);
    for (auto& frame : hist_) frame.fill(0.f);
}

Observation799 ObservationBuilder::build(const RobotSample& s, const Command3& cmd) {
    // Validate *before* changing phase/history so rejected frames leave state intact.
    requireFinite(s.joint_q, "joint_q");
    requireFinite(s.joint_dq, "joint_dq");
    requireFinite(s.imu_quat_wxyz, "imu_quat_wxyz");
    requireFinite(s.imu_gyro_xyz, "imu_gyro_xyz");
    requireFinite(s.ee_local_xyz, "ee_local_xyz");
    requireFinite(cmd, "command");

    const auto& q = s.imu_quat_wxyz;
    const double norm = std::sqrt(
        static_cast<double>(q[0])*q[0] + static_cast<double>(q[1])*q[1] +
        static_cast<double>(q[2])*q[2] + static_cast<double>(q[3])*q[3]);
    if (norm < 0.9 || norm > 1.1) {
        throw std::invalid_argument("IMU quaternion norm outside [0.9,1.1]");
    }
    const double w = q[0] / norm;
    const double x = q[1] / norm;
    const double y = q[2] / norm;
    const double z = q[3] / norm;
    const float roll = static_cast<float>(std::atan2(
        2.0*(w*x + y*z), 1.0 - 2.0*(x*x+y*y)));
    const float pitch = static_cast<float>(std::asin(std::clamp(
        2.0*(w*y - z*x), -1.0, 1.0)));

    // Python _gait() adds step_dt*freq before deciding whether to stop.
    double next_phase = std::fmod(gait_phase_ +
        static_cast<double>(cfg_.control_dt) * cfg_.gait_frequency_hz, 1.0);
    const bool walking = std::any_of(cmd.begin(), cmd.end(),
        [](float value) { return std::abs(value) > 0.1f; });
    if (!walking) next_phase = 0.0;

    Proprio71 p{};
    p[0] = roll;
    p[1] = pitch;
    for (std::size_t i = 0; i < 3; ++i) {
        p[2+i] = s.imu_gyro_xyz[i] * cfg_.ang_vel_scale;
    }
    for (std::size_t i = 0; i < kDof; ++i) {
        p[5+i] = (s.joint_q[i] - cfg_.default_joint_pos[i]) * cfg_.dof_pos_scale;
        p[23+i] = s.joint_dq[i] * cfg_.dof_vel_scale;
    }
    for (std::size_t i = 0; i < kLegDof; ++i) {
        p[41+i] = last_action_[i];
    }
    // p[53:57] foot contacts zeros; p[63:66] orientation goal zeros.
    for (std::size_t i = 0; i < 3; ++i) {
        p[57+i] = cmd[i] * cfg_.cmd_scale[i];
        p[60+i] = s.ee_local_xyz[i];
    }
    p[66] = static_cast<float>(next_phase);
    const std::array<double, 4> shift{0.5, 0.0, 0.0, 0.5};
    for (std::size_t i = 0; i < 4; ++i) {
        p[67+i] = static_cast<float>(std::sin(2.0*kPi*(next_phase + shift[i])));
    }

    Observation799 obs{};
    std::copy(p.begin(), p.end(), obs.begin());
    obs[kProprioDim + 5] = 1.f; // ALORE priv[5] friction placeholder.
    for (std::size_t h = 0; h < kHistoryLen; ++h) {
        std::copy(hist_[h].begin(), hist_[h].end(),
                  obs.begin() + kProprioDim + kPrivDim + h*kProprioDim);
    }

    // Side effects occur only after successful observation construction.
    if (step_count_ == 0) {
        for (auto& frame : hist_) frame = p;
    } else {
        for (std::size_t h = 0; h+1 < kHistoryLen; ++h) hist_[h] = hist_[h+1];
        hist_[kHistoryLen-1] = p;
    }
    last_proprio_ = p;
    gait_phase_ = next_phase;
    ++step_count_;
    return obs;
}

void ObservationBuilder::acceptPolicyAction(const Joint18& raw_action) {
    requireFinite(raw_action, "raw policy action");
    for (std::size_t i = 0; i < kDof; ++i) {
        last_action_[i] = std::clamp(raw_action[i],
                                    -cfg_.max_abs_action, cfg_.max_abs_action);
    }
}

} // namespace interactive_b2z1
