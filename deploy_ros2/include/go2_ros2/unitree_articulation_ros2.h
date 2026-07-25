#pragma once

#include "go2_ros2/ros2_lowcmd_lowstate.h"
#include "isaaclab/assets/articulation/articulation.h"

namespace go2_ros2
{

class Ros2BaseArticulation : public isaaclab::Articulation
{
public:
    explicit Ros2BaseArticulation(Ros2LowState::SharedPtr lowstate_)
    : lowstate(std::move(lowstate_))
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        for (int i = 0; i < 3; ++i)
        {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state.gyroscope[i];
        }
        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state.quaternion[0],
            lowstate->msg_.imu_state.quaternion[1],
            lowstate->msg_.imu_state.quaternion[2],
            lowstate->msg_.imu_state.quaternion[3]);
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;

        for (size_t i = 0; i < data.joint_ids_map.size(); ++i)
        {
            const auto joint_id = static_cast<size_t>(data.joint_ids_map[i]);
            data.joint_pos[i] = lowstate->msg_.motor_state[joint_id].q;
            data.joint_vel[i] = lowstate->msg_.motor_state[joint_id].dq;
        }
    }

    Ros2LowState::SharedPtr lowstate;
};

} // namespace go2_ros2
