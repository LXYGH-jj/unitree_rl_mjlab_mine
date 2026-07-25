// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "FSMState.h"
#include "LinearInterpolator.h"

class State_SlowLieDown : public FSMState
{
public:
    State_SlowLieDown(int state, std::string state_string = "SlowLieDown")
    : FSMState(state, state_string)
    {
        ts_ = param::config["FSM"][state_string]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"][state_string]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());

        registered_checks.emplace_back(
            std::make_pair(
                [&]()->bool{ return done_; },
                FSMStringMap.right.at("Passive")
            )
        );
    }

    void enter()
    {
        done_ = false;

        static auto kp = param::config["FSM"]["SlowLieDown"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["SlowLieDown"]["kd"].as<std::vector<float>>();
        const int joint_count = std::min<int>(lowcmd_joint_count_for_whole_body(), kp.size());
        for (int i = 0; i < joint_count; ++i)
        {
            auto& motor = lowcmd->msg_.motor_cmd()[i];
            motor.kp() = kp[i];
            motor.kd() = kd[i];
            motor.dq() = 0;
            motor.tau() = 0;
        }

        std::vector<float> q0;
        q0.reserve(kp.size());
        for (int i = 0; i < joint_count; ++i)
        {
            q0.push_back(lowstate->msg_.motor_state()[i].q());
        }
        qs_[0] = q0;
        t0_ = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3;
    }

    void run()
    {
        const float t = (double)unitree::common::GetCurrentTimeMillisecond() * 1e-3 - t0_;
        const auto q = linear_interpolate(t, ts_, qs_);

        for (int i = 0; i < q.size() && i < lowcmd_joint_count_for_whole_body(); ++i)
        {
            lowcmd->msg_.motor_cmd()[i].q() = q[i];
        }

        run_z1_diagnostic_if_active("SlowLieDown");
        done_ = t >= ts_.back();
    }

private:
    double t0_ = 0.0;
    bool done_ = false;
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

REGISTER_FSM(State_SlowLieDown)
