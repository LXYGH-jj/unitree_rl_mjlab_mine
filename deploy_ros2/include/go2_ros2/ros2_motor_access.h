#pragma once

#include <cstddef>

#include "go2_ros2/ros2_lowcmd_lowstate.h"

namespace go2_ros2::motor
{

inline size_t cmd_size(const Ros2LowCmd& cmd)
{
    return cmd.msg_.motor_cmd.size();
}

inline void set_mode(Ros2LowCmd& cmd, size_t i, int mode)
{
    cmd.msg_.motor_cmd[i].mode = static_cast<uint8_t>(mode);
}

inline void set_q(Ros2LowCmd& cmd, size_t i, float q)
{
    cmd.msg_.motor_cmd[i].q = q;
}

inline void set_dq(Ros2LowCmd& cmd, size_t i, float dq)
{
    cmd.msg_.motor_cmd[i].dq = dq;
}

inline void set_kp(Ros2LowCmd& cmd, size_t i, float kp)
{
    cmd.msg_.motor_cmd[i].kp = kp;
}

inline void set_kd(Ros2LowCmd& cmd, size_t i, float kd)
{
    cmd.msg_.motor_cmd[i].kd = kd;
}

inline void set_tau(Ros2LowCmd& cmd, size_t i, float tau)
{
    cmd.msg_.motor_cmd[i].tau = tau;
}

inline float cmd_q(const Ros2LowCmd& cmd, size_t i)
{
    return cmd.msg_.motor_cmd[i].q;
}

inline float state_q(const Ros2LowState& state, size_t i)
{
    return state.msg_.motor_state[i].q;
}

inline float state_dq(const Ros2LowState& state, size_t i)
{
    return state.msg_.motor_state[i].dq;
}

} // namespace go2_ros2::motor
