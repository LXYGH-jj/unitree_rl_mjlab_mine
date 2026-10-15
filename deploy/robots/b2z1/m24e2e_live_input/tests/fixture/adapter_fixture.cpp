#include "interactive_feedback_adapter.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace interactive_b2z1 {
Joint12 b2MotorToPolicy(const Joint12& q){Joint12 out{};for(unsigned i=0;i<12;++i)out[i]=q[kPolicyToB2Motor[i]];return out;}
Command3 eeGoalObservationFromBaseTarget(const Command3& goal,const std::array<float,4>&,const Command3& offset){return {goal[0]-offset[0],goal[1]-offset[1],goal[2]-offset[2]};}
AdaptedInput adaptFeedback(const FeedbackSnapshot& s,MonoTime now,const FeedbackAdapterConfig& cfg){
 if(!s.b2_valid||!s.z1_valid||s.b2_received_at==MonoTime{}||s.z1_received_at==MonoTime{}||
  s.b2_received_at>now||s.z1_received_at>now||now-s.b2_received_at>cfg.max_b2_age||
  now-s.z1_received_at>cfg.max_z1_age||
  (s.b2_received_at>s.z1_received_at?s.b2_received_at-s.z1_received_at:s.z1_received_at-s.b2_received_at)>cfg.max_interstream_skew)
 throw std::runtime_error("stale/skew");
 AdaptedInput o{};const auto q=b2MotorToPolicy(s.b2_motor_q),dq=b2MotorToPolicy(s.b2_motor_dq);
 for(unsigned i=0;i<12;i++){o.robot.joint_q[i]=q[i];o.robot.joint_dq[i]=dq[i];}
 for(unsigned i=0;i<6;i++){o.robot.joint_q[12+i]=s.z1_q[i];o.robot.joint_dq[12+i]=s.z1_dq[i];}
 o.robot.imu_quat_wxyz=s.imu_quat_wxyz;o.robot.imu_gyro_xyz=s.imu_gyro_xyz;
 o.robot.ee_local_xyz=eeGoalObservationFromBaseTarget(s.ee_goal_base_xyz,s.imu_quat_wxyz,cfg.nominal_arm_offset_yaw_frame);
 o.command=s.base_command;return o;
}
}
