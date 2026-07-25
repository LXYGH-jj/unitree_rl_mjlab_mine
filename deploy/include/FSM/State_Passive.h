// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"

class State_Passive : public FSMState
{
public:
    State_Passive(int state, std::string state_string = "Passive") 
    : FSMState(state, state_string) 
    {
        auto motor_mode = param::config["FSM"]["Passive"]["mode"];
        if(motor_mode.IsDefined())
        {
            auto values = motor_mode.as<std::vector<int>>();
            for(int i(0); i<values.size(); ++i)
            {
                lowcmd->msg_.motor_cmd()[i].mode() = values[i];
            }
        }
    } 

    void enter()
    {
        // set gain
        static auto kd = param::config["FSM"]["Passive"]["kd"].as<std::vector<float>>();
        const int joint_count = std::min<int>(lowcmd_joint_count_for_whole_body(), kd.size());
        for(int i(0); i < joint_count; ++i)
        {
            auto & motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = 0;
            motor.kd() = kd[i];
            motor.dq() = 0;
            motor.tau() = 0;
        }
    }

    void run()
    {
        for(int i(0); i < lowcmd_joint_count_for_whole_body(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].q() = lowstate->msg_.motor_state()[i].q();
        }

        run_z1_diagnostic_if_active("Passive");
    }
};

REGISTER_FSM(State_Passive)
