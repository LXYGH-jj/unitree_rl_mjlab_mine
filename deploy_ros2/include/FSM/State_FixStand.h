#pragma once

#include <chrono>

#include "FSM/FSMState.h"
#include "LinearInterpolator.h"
#include "go2_ros2/ros2_motor_access.h"

class State_FixStand : public FSMState
{
public:
    State_FixStand(int state, std::string state_string = "FixStand")
    : FSMState(state, state_string)
    {
        ts_ = param::config["FSM"]["FixStand"]["ts"].as<std::vector<float>>();
        qs_ = param::config["FSM"]["FixStand"]["qs"].as<std::vector<std::vector<float>>>();
        assert(ts_.size() == qs_.size());
    }

    void enter() override
    {
        static auto kp = param::config["FSM"]["FixStand"]["kp"].as<std::vector<float>>();
        static auto kd = param::config["FSM"]["FixStand"]["kd"].as<std::vector<float>>();
        for (size_t i = 0; i < kp.size(); ++i)
        {
            go2_ros2::motor::set_kp(*lowcmd, i, kp[i]);
            go2_ros2::motor::set_kd(*lowcmd, i, kd[i]);
            go2_ros2::motor::set_dq(*lowcmd, i, 0.0f);
            go2_ros2::motor::set_tau(*lowcmd, i, 0.0f);
        }

        std::vector<float> q0;
        for (size_t i = 0; i < kp.size(); ++i)
        {
            q0.push_back(go2_ros2::motor::cmd_q(*lowcmd, i));
        }
        qs_[0] = q0;
        t0_ = now_sec();
    }

    void run() override
    {
        const float t = now_sec() - t0_;
        auto q = linear_interpolate(t, ts_, qs_);
        for (size_t i = 0; i < q.size(); ++i)
        {
            go2_ros2::motor::set_q(*lowcmd, i, q[i]);
        }
    }

private:
    static double now_sec()
    {
        using clock = std::chrono::steady_clock;
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    }

    double t0_{0.0};
    std::vector<float> ts_;
    std::vector<std::vector<float>> qs_;
};

REGISTER_FSM(State_FixStand)
