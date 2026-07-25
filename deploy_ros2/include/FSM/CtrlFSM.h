// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <spdlog/spdlog.h>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "FSM/BaseState.h"

class CtrlFSM
{
public:
    explicit CtrlFSM(YAML::Node cfg)
    {
        auto fsms = cfg["_"];
        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            const std::string fsm_name = it->first.as<std::string>();
            const int id = it->second["id"].as<int>();
            FSMStringMap.insert({id, fsm_name});
        }

        for (auto it = fsms.begin(); it != fsms.end(); ++it)
        {
            const std::string fsm_name = it->first.as<std::string>();
            const int id = it->second["id"].as<int>();
            const std::string fsm_type = it->second["type"] ? it->second["type"].as<std::string>() : fsm_name;
            auto fsm_class = getFsmMap().find("State_" + fsm_type);
            if (fsm_class == getFsmMap().end())
            {
                throw std::runtime_error("FSM: Unknown FSM type " + fsm_type);
            }
            add(fsm_class->second(id, fsm_name));
        }
    }

    void start()
    {
        currentState = states.at(0);
        currentState->enter();
        running_ = true;
        fsm_thread_ = std::thread([this] {
            using clock = std::chrono::steady_clock;
            const auto period = std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(dt_));
            auto next = clock::now() + period;
            while (running_)
            {
                run_();
                std::this_thread::sleep_until(next);
                next += period;
            }
        });
        spdlog::info("FSM: Start {}", currentState->getStateString());
    }

    ~CtrlFSM()
    {
        running_ = false;
        if (fsm_thread_.joinable())
        {
            fsm_thread_.join();
        }
    }

    void add(std::shared_ptr<BaseState> state)
    {
        for (auto& s : states)
        {
            if (s->isState(state->getState()))
            {
                throw std::runtime_error("FSM: duplicated state " + state->getStateString());
            }
        }
        states.push_back(std::move(state));
    }

    std::vector<std::shared_ptr<BaseState>> states;

private:
    const double dt_ = 0.001;

    void run_()
    {
        currentState->pre_run();
        currentState->run();
        currentState->post_run();

        int nextStateMode = 0;
        for (auto& check : currentState->registered_checks)
        {
            if (check.first())
            {
                nextStateMode = check.second;
                break;
            }
        }

        if (nextStateMode != 0 && !currentState->isState(nextStateMode))
        {
            for (auto& state : states)
            {
                if (state->isState(nextStateMode))
                {
                    spdlog::info("FSM: Change state from {} to {}", currentState->getStateString(), state->getStateString());
                    currentState->exit();
                    currentState = state;
                    currentState->enter();
                    break;
                }
            }
        }
    }

    std::shared_ptr<BaseState> currentState;
    std::atomic_bool running_{false};
    std::thread fsm_thread_;
};
