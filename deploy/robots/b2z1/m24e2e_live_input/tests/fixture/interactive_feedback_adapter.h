#pragma once
#include "interactive_observation.h"
#include <chrono>
namespace interactive_b2z1 {
using MonoClock=std::chrono::steady_clock;using MonoTime=MonoClock::time_point;using Arm6=std::array<float,6>;
struct FeedbackSnapshot {Joint12 b2_motor_q{},b2_motor_dq{};Arm6 z1_q{},z1_dq{};std::array<float,4> imu_quat_wxyz{1,0,0,0};Command3 imu_gyro_xyz{},ee_goal_base_xyz{},base_command{};MonoTime b2_received_at{},z1_received_at{};bool b2_valid{false},z1_valid{false};};
struct FeedbackAdapterConfig {std::chrono::milliseconds max_b2_age{100},max_z1_age{100},max_interstream_skew{50};Command3 nominal_arm_offset_yaw_frame{.20f,0.f,.09f};};
struct AdaptedInput {RobotSample robot{};Command3 command{};};
Command3 eeGoalObservationFromBaseTarget(const Command3&,const std::array<float,4>&,const Command3&);
AdaptedInput adaptFeedback(const FeedbackSnapshot&,MonoTime,const FeedbackAdapterConfig& cfg={});
}
