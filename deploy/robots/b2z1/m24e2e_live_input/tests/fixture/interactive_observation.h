#pragma once
// Minimal API-compatible fixture for container-only test. On user's PC the REAL project is used.
#include <array>
#include <cstdint>
#include <cstddef>
namespace interactive_b2z1 {
constexpr std::size_t kLegDof=12,kDof=18,kProprioDim=71,kPrivDim=18,kHistoryLen=10,kObsDim=799;
using Joint18=std::array<float,18>; using Joint12=std::array<float,12>; using Command3=std::array<float,3>;
using Proprio71=std::array<float,71>;using Observation799=std::array<float,799>;
constexpr std::array<std::size_t,12> kPolicyToB2Motor={3,4,5,0,1,2,9,10,11,6,7,8};
Joint12 b2MotorToPolicy(const Joint12&);
struct RobotSample{
 Joint18 joint_q{},joint_dq{};std::array<float,4> imu_quat_wxyz{1,0,0,0};Command3 imu_gyro_xyz{},ee_local_xyz{};
};
struct ObservationConfig {Joint18 default_joint_pos{};Command3 cmd_scale{2,2,.25f};float control_dt{.02f},gait_frequency_hz{2.f},dof_pos_scale{1},dof_vel_scale{.05f},ang_vel_scale{.25f},max_abs_action{100};static ObservationConfig baseline();};
class ObservationBuilder{public:explicit ObservationBuilder(ObservationConfig cfg=ObservationConfig::baseline());Observation799 build(const RobotSample& s,const Command3& cmd);void acceptPolicyAction(const Joint18& a);private:ObservationConfig cfg_;Joint18 last_action_{};std::array<Proprio71,10> hist_{};double gait_phase_{};std::uint64_t step_count_{};};
}
