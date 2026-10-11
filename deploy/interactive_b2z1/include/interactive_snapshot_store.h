#pragma once

#include "interactive_feedback_adapter.h"

#include <mutex>

namespace interactive_b2z1 {

// Thread-safe copy-only store. No SDK / ROS / DDS dependency or motor interface.
// Time points must be taken at the actual feedback reception callback on this host.
// In the loopback simulator each synthetic B2+Z1 sample originates in one LowState.
class SnapshotStore final {
public:
    void updateB2(const Joint12& motor_q, const Joint12& motor_dq,
                  const std::array<float, 4>& imu_quat_wxyz,
                  const Command3& imu_gyro_xyz, MonoTime received_at);
    void updateZ1(const Arm6& q, const Arm6& dq, MonoTime received_at);
    [[nodiscard]] FeedbackSnapshot copy(const Command3& base_command,
                                        const Command3& ee_goal_base_xyz) const;
    void reset();

private:
    mutable std::mutex mutex_;
    FeedbackSnapshot latest_{};
};

} // namespace interactive_b2z1
