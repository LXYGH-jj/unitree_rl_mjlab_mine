#include "interactive_snapshot_store.h"

#include <stdexcept>

namespace interactive_b2z1 {

void SnapshotStore::updateB2(const Joint12& q, const Joint12& dq,
                             const std::array<float,4>& quat,
                             const Command3& gyro, MonoTime received_at) {
    if (received_at == MonoTime{})
        throw std::invalid_argument("B2 timestamp must be nonzero");
    std::lock_guard<std::mutex> guard(mutex_);
    if (latest_.b2_valid && received_at <= latest_.b2_received_at)
        throw std::invalid_argument("B2 timestamp must advance");
    latest_.b2_motor_q = q;
    latest_.b2_motor_dq = dq;
    latest_.imu_quat_wxyz = quat;
    latest_.imu_gyro_xyz = gyro;
    latest_.b2_received_at = received_at;
    latest_.b2_valid = true;
}

void SnapshotStore::updateZ1(const Arm6& q, const Arm6& dq, MonoTime received_at) {
    if (received_at == MonoTime{})
        throw std::invalid_argument("Z1 timestamp must be nonzero");
    std::lock_guard<std::mutex> guard(mutex_);
    if (latest_.z1_valid && received_at <= latest_.z1_received_at)
        throw std::invalid_argument("Z1 timestamp must advance");
    latest_.z1_q = q;
    latest_.z1_dq = dq;
    latest_.z1_received_at = received_at;
    latest_.z1_valid = true;
}

FeedbackSnapshot SnapshotStore::copy(const Command3& cmd, const Command3& goal) const {
    std::lock_guard<std::mutex> guard(mutex_);
    auto snap = latest_;
    snap.base_command = cmd;
    snap.ee_goal_base_xyz = goal;
    return snap;
}

void SnapshotStore::reset() {
    std::lock_guard<std::mutex> guard(mutex_);
    latest_ = FeedbackSnapshot{};
}

} // namespace interactive_b2z1
