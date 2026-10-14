#pragma once
// Registered alias of the SIM-only InteractiveArmIK FSM, with an opt-in
// M2.4d4 mock Z1 safety gate. This is NEVER a real Z1 JOINTCTRL interface.
#include "State_InteractiveArmIK.h"
class State_JointCtrlMockSim final : public State_InteractiveArmIK {
public:
    State_JointCtrlMockSim(int id, std::string name)
        : State_InteractiveArmIK(id, name) {
        if (name != "JointCtrlMockSim")
            throw std::runtime_error("M2.4d5 FSM name must be JointCtrlMockSim");
    }
};
REGISTER_FSM(State_JointCtrlMockSim)
