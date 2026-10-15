#pragma once
// Adapter from actual B2 12-slot feedback and real/mock Z1 IPC frame into
// the *existing* Interactive 799-D observation layout. No DDS or motor writes.
#include "interactive_feedback_adapter.h"
#include "interactive_observation.h"
#include "z1_ipc_client.h"
namespace m24e2e {
struct B2State {
    interactive_b2z1::Joint12 motor_q{}, motor_dq{}; // motor FL FR RL RR
    std::array<float,4> quat_wxyz{1,0,0,0};
    interactive_b2z1::Command3 gyro{};
    interactive_b2z1::MonoTime packet_received_at{}; // B2 DDS callback stamp, NOT read-time
    bool valid{false};
};
// Reuses training-frame EE transform and original q/dq ordering. Refuses
// stale or contradictory data. This is not a physical safety controller.
interactive_b2z1::AdaptedInput makeLiveInput(
    const B2State& b2, const ArmState& z1,
    const interactive_b2z1::Command3& base_command,
    const interactive_b2z1::Command3& ee_goal_base_xyz,
    interactive_b2z1::MonoTime now,
    Mode required_mode,
    const interactive_b2z1::FeedbackAdapterConfig& cfg={});
}
