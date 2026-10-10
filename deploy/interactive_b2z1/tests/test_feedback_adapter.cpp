#include "interactive_feedback_adapter.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

using namespace interactive_b2z1;

void expect(bool cond, const char* msg) {
    if (!cond) throw std::runtime_error(msg);
}
template<class F> void expectReject(F&& f, const char* msg) {
    bool rejected = false;
    try { f(); } catch (const std::exception&) { rejected = true; }
    expect(rejected, msg);
}
int main() {
  const auto now = MonoClock::now();
  FeedbackSnapshot s;
  s.b2_valid = s.z1_valid = true;
  s.b2_received_at = now - std::chrono::milliseconds(10);
  s.z1_received_at = now - std::chrono::milliseconds(8);
  for (int i=0;i<12;++i) { s.b2_motor_q[i]=static_cast<float>(i); s.b2_motor_dq[i]=i*0.5f; }
  for (int i=0;i<6;++i) { s.z1_q[i]=30.f+i; s.z1_dq[i]=40.f+i; }
  s.imu_quat_wxyz = {1.f, 0.f, 0.f, 0.f};
  s.ee_goal_base_xyz = {0.5f,0.1f,0.4f};
  s.base_command = {0.2f,0.f,0.f};
  auto out = adaptFeedback(s, now);
  const int expected[12] = {3,4,5,0,1,2,9,10,11,6,7,8};
  for (int i=0;i<12;++i) {
    expect(out.robot.joint_q[i]==static_cast<float>(expected[i]), "wrong B2 mapping");
    expect(out.robot.joint_dq[i]==expected[i]*0.5f,"wrong B2 vel mapping");
  }
  for (int i=0;i<6;++i) {
    expect(out.robot.joint_q[12+i]==30.f+i,"wrong Z1 positions");
    expect(out.robot.joint_dq[12+i]==40.f+i,"wrong Z1 velocities");
  }
  expect(std::abs(out.robot.ee_local_xyz[0]-.3f)<1e-6f,"EE nominal offset X");
  expect(std::abs(out.robot.ee_local_xyz[1]-.1f)<1e-6f,"EE nominal offset Y");
  expect(std::abs(out.robot.ee_local_xyz[2]-.31f)<1e-6f,"EE nominal offset Z");
  // 90-degree roll: R_base^T * R_yaw * [0.2,0,0.09] = [0.2, 0.09, 0]
  constexpr float r = 0.7071067811865475f;
  s.imu_quat_wxyz={r,r,0.f,0.f};
  out=adaptFeedback(s,now);
  expect(std::abs(out.robot.ee_local_xyz[0]-.3f)<1e-5f,"roll transform X");
  expect(std::abs(out.robot.ee_local_xyz[1]-.01f)<1e-5f,"roll transform Y");
  expect(std::abs(out.robot.ee_local_xyz[2]-.4f)<1e-5f,"roll transform Z");
  s.imu_quat_wxyz={1.f,0.f,0.f,0.f};
  s.b2_received_at=now-std::chrono::milliseconds(200);
  expectReject([&]{adaptFeedback(s,now);},"stale B2 must reject");
  s.b2_received_at=now-std::chrono::milliseconds(10);
  s.z1_received_at=now-std::chrono::milliseconds(200);
  expectReject([&]{adaptFeedback(s,now);},"stale Z1 must reject");
  // Both individually fresh, but asynchronous by >50ms: reject cross-stream skew.
  s.b2_received_at=now-std::chrono::milliseconds(90);
  s.z1_received_at=now-std::chrono::milliseconds(8);
  expectReject([&]{adaptFeedback(s,now);},"interstream skew must reject");
  s.b2_received_at=now-std::chrono::milliseconds(10);
  s.z1_received_at=now-std::chrono::milliseconds(8);
  s.z1_valid=false;
  expectReject([&]{adaptFeedback(s,now);},"missing Z1 must reject");
  s.z1_valid=true;
  s.b2_motor_q[5]=std::numeric_limits<float>::quiet_NaN();
  expectReject([&]{adaptFeedback(s,now);},"nan joint must reject");
  s.b2_motor_q[5]=5.f;
  s.imu_quat_wxyz={0.f,0.f,0.f,0.f};
  expectReject([&]{adaptFeedback(s,now);},"zero quaternion must reject");
  s.imu_quat_wxyz={1.f,0.f,0.f,0.f};
  s.z1_received_at=now+std::chrono::milliseconds(1);
  expectReject([&]{adaptFeedback(s,now);},"future feedback must reject");
  std::cout << "M1.4b FEEDBACK ADAPTER: PASS\n";
}
