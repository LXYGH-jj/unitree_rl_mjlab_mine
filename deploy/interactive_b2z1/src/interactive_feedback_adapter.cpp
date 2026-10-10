#include "interactive_feedback_adapter.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace interactive_b2z1 {
namespace {
template <std::size_t N>
void requireFinite(const std::array<float, N>& a, const char* name) {
    for (float f : a) if (!std::isfinite(f))
        throw std::invalid_argument(std::string(name) + " contains NaN/Inf");
}
void requireFresh(MonoTime stamp, MonoTime now, std::chrono::milliseconds max_age,
                  const char* name) {
    if (max_age.count() <= 0 || stamp == MonoTime{} || stamp > now ||
        now - stamp > max_age)
        throw std::runtime_error(std::string(name) + " feedback invalid or stale");
}
std::array<double, 4> unitQuat(const std::array<float, 4>& q) {
    requireFinite(q, "imu quaternion");
    double n = 0.;
    for (float v : q) n += static_cast<double>(v) * v;
    n = std::sqrt(n);
    if (n < 0.9 || n > 1.1)
        throw std::invalid_argument("imu quaternion norm outside [0.9,1.1]");
    return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}
Command3 quatRotateInverse(const std::array<double, 4>& q, const Command3& v) {
    // world -> base: R(q)^T v. Quaternion is wxyz and represents base->world.
    const double w=q[0], x=q[1], y=q[2], z=q[3];
    const double xx=x*x, yy=y*y, zz=z*z, xy=x*y, xz=x*z, yz=y*z;
    const double wx=w*x, wy=w*y, wz=w*z;
    return {
        static_cast<float>((1.-2.*(yy+zz))*v[0]+(2.*(xy+wz))*v[1]+(2.*(xz-wy))*v[2]),
        static_cast<float>((2.*(xy-wz))*v[0]+(1.-2.*(xx+zz))*v[1]+(2.*(yz+wx))*v[2]),
        static_cast<float>((2.*(xz+wy))*v[0]+(2.*(yz-wx))*v[1]+(1.-2.*(xx+yy))*v[2])
    };
}
} // namespace

Command3 eeGoalObservationFromBaseTarget(const Command3& goal_base,
                                         const std::array<float,4>& quat_wxyz,
                                         const Command3& nominal_arm_offset_yaw_frame) {
    requireFinite(goal_base, "EE goal");
    requireFinite(nominal_arm_offset_yaw_frame, "nominal arm mount offset");
    const auto q = unitQuat(quat_wxyz);
    const double w=q[0],x=q[1],y=q[2],z=q[3];
    const double yaw = std::atan2(2.*(w*z+x*y), 1.-2.*(y*y+z*z));
    const double c = std::cos(yaw), s = std::sin(yaw);
    const Command3 nominal_w{
        static_cast<float>(c*nominal_arm_offset_yaw_frame[0]-s*nominal_arm_offset_yaw_frame[1]),
        static_cast<float>(s*nominal_arm_offset_yaw_frame[0]+c*nominal_arm_offset_yaw_frame[1]),
        nominal_arm_offset_yaw_frame[2]
    };
    const auto nominal_b = quatRotateInverse(q, nominal_w);
    return {goal_base[0]-nominal_b[0], goal_base[1]-nominal_b[1],
            goal_base[2]-nominal_b[2]};
}

AdaptedInput adaptFeedback(const FeedbackSnapshot& s, MonoTime now,
                           const FeedbackAdapterConfig& cfg) {
    if (!s.b2_valid || !s.z1_valid)
        throw std::runtime_error("missing B2 or Z1 feedback");
    requireFresh(s.b2_received_at, now, cfg.max_b2_age, "B2");
    requireFresh(s.z1_received_at, now, cfg.max_z1_age, "Z1");
    if (cfg.max_interstream_skew.count() < 0 ||
        (s.b2_received_at > s.z1_received_at
            ? s.b2_received_at-s.z1_received_at
            : s.z1_received_at-s.b2_received_at) > cfg.max_interstream_skew)
        throw std::runtime_error("B2/Z1 feedback timestamps too far apart");
    requireFinite(s.b2_motor_q, "B2 joint_q");
    requireFinite(s.b2_motor_dq, "B2 joint_dq");
    requireFinite(s.z1_q, "Z1 joint_q");
    requireFinite(s.z1_dq, "Z1 joint_dq");
    requireFinite(s.imu_gyro_xyz, "IMU gyro");
    requireFinite(s.base_command, "command");
    const auto leg_q = b2MotorToPolicy(s.b2_motor_q);
    const auto leg_dq = b2MotorToPolicy(s.b2_motor_dq);
    AdaptedInput out;
    for (std::size_t i=0; i<12; ++i) {
        out.robot.joint_q[i] = leg_q[i];
        out.robot.joint_dq[i] = leg_dq[i];
    }
    for (std::size_t i=0; i<6; ++i) {
        out.robot.joint_q[12+i] = s.z1_q[i];
        out.robot.joint_dq[12+i] = s.z1_dq[i];
    }
    out.robot.imu_quat_wxyz = s.imu_quat_wxyz;
    out.robot.imu_gyro_xyz = s.imu_gyro_xyz;
    out.robot.ee_local_xyz = eeGoalObservationFromBaseTarget(
        s.ee_goal_base_xyz, s.imu_quat_wxyz, cfg.nominal_arm_offset_yaw_frame);
    out.command = s.base_command;
    return out;
}
} // namespace interactive_b2z1
