#include "b2z1_live_input.h"
#include <cmath>
#include <stdexcept>
namespace m24e2e {
interactive_b2z1::AdaptedInput makeLiveInput(
    const B2State& b2,const ArmState& z1,
    const interactive_b2z1::Command3& cmd,
    const interactive_b2z1::Command3& goal,
    interactive_b2z1::MonoTime now,Mode required_mode,
    const interactive_b2z1::FeedbackAdapterConfig& cfg) {
    if(!b2.valid || z1.mode!=required_mode)throw std::runtime_error("B2 invalid or Z1 worker mode unexpected");
    if(z1.ipc_received_at==Clock::time_point{})throw std::runtime_error("Z1 never responded");
    interactive_b2z1::FeedbackSnapshot snap{};
    snap.b2_motor_q=b2.motor_q;snap.b2_motor_dq=b2.motor_dq;
    snap.z1_q={};snap.z1_dq={};
    for(std::size_t i=0;i<6;i++){
        if(!std::isfinite(z1.q[i])||!std::isfinite(z1.dq[i]))throw std::runtime_error("Z1 state nonfinite");
        snap.z1_q[i]=static_cast<float>(z1.q[i]);
        snap.z1_dq[i]=static_cast<float>(z1.dq[i]);
    }
    snap.imu_quat_wxyz=b2.quat_wxyz;snap.imu_gyro_xyz=b2.gyro;
    snap.base_command=cmd;snap.ee_goal_base_xyz=goal;
    snap.b2_received_at=b2.packet_received_at;
    snap.z1_received_at=z1.ipc_received_at; // IPC reply time only; NOT SDK packet RX time
    snap.b2_valid=true;snap.z1_valid=true;
    return interactive_b2z1::adaptFeedback(snap,now,cfg);
}
}
