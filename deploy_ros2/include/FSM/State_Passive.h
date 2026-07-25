#pragma once

#include "FSM/FSMState.h"
#include "go2_ros2/ros2_motor_access.h"

class State_Passive : public FSMState
{
public:
    State_Passive(int state, std::string state_string = "Passive")
    : FSMState(state, state_string)
    {
        auto motor_mode = param::config["FSM"]["Passive"]["mode"];
        if (motor_mode.IsDefined())
        {
            auto values = motor_mode.as<std::vector<int>>();
            for (size_t i = 0; i < values.size(); ++i)
            {
                go2_ros2::motor::set_mode(*lowcmd, i, values[i]);
            }
        }
    }

    void enter() override
    {
        static auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
        for (size_t i = 0; i < kd.size(); ++i)
        {
            go2_ros2::motor::set_kp(*lowcmd, i, 0.0f);
            go2_ros2::motor::set_kd(*lowcmd, i, kd[i]);
            go2_ros2::motor::set_dq(*lowcmd, i, 0.0f);
            go2_ros2::motor::set_tau(*lowcmd, i, 0.0f);
        }
    }

    void run() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        const auto n = std::min(go2_ros2::motor::cmd_size(*lowcmd), lowstate->msg_.motor_state.size());
        for (size_t i = 0; i < n; ++i)
        {
            go2_ros2::motor::set_q(*lowcmd, i, go2_ros2::motor::state_q(*lowstate, i));
        }
    }
};

REGISTER_FSM(State_Passive)
