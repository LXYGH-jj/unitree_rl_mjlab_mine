#pragma once

#include <atomic>
#include <thread>

#include "FSM/FSMState.h"
#include "isaaclab/envs/mdp/actions/joint_actions.h"
#include "isaaclab/envs/mdp/terminations.h"

class State_RLBase : public FSMState
{
public:
    State_RLBase(int state_mode, std::string state_string);

    void enter() override;
    void run() override;
    void exit() override;

private:
    std::unique_ptr<isaaclab::ManagerBasedRLEnv> env;
    std::thread policy_thread;
    std::atomic_bool policy_thread_running{false};
};
