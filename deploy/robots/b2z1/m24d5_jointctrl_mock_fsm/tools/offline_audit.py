#!/usr/bin/env python3
from pathlib import Path
import sys
repo=Path(sys.argv[1]).resolve()
base=repo/'deploy/robots/b2z1'
main=(base/'main.cpp').read_text()
cpp=(base/'src/State_InteractiveArmIK.cpp').read_text()
h=(base/'include/State_InteractiveArmIK.h').read_text()
cm=(base/'CMakeLists.txt').read_text()
cfg=(base/'config/config_interactive_jointctrl_mock_m24d5_sim.yaml').read_text()
mod=base/'m24d5_jointctrl_mock_fsm'
checks={
 'pre-DDS hardware refusal': 'M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED' in main and main.find('M2.4d5 refuses non-loopback') < main.find('ChannelFactory::Instance()->Init'),
 'FSM registry alias': '#include "State_JointCtrlMockSim.h"' in main and 'REGISTER_FSM(State_JointCtrlMockSim)' in (mod/'include/State_JointCtrlMockSim.h').read_text(),
 'model joint limits from existing IK model': 'getJointQMin()' in cpp and 'getJointQMax()' in cpp,
 'mock opt-in only': 'getStateString() == "JointCtrlMockSim"' in cpp and 'mock_sim_mode_' in h,
 'synthetic gate connected to motor_step': 'mock_sim_gate_->step(sample.tick_ms,q,dq,goal,Clock::now())' in cpp and 'motors[12+i].q(static_cast<float>(arm_cmd_[i]))' in cpp,
 'existing policy observation reused': 'adaptFeedback(snapshots_.copy(velocity_cmd_,ee_goal)' in cpp and 'pipeline_->step(input.robot,input.command)' in cpp,
 'existing B2 leg output retained': 'motors[i].q(commanded_[i])' in cpp,
 'M2.4d4 contract links through CMake': 'm24d4_z1_transport_contract/src/z1_transport_contract.cpp' in cm,
 'separate 100% mock config': 'jointctrl_mock_sim:\n  enabled: true\n  sim_only: true\n  no_vendor_sdk_io: true' in cfg and 'JointCtrlMockSim: X.on_pressed' in cfg and 'InteractiveArmIK: X.on_pressed' not in cfg,
 'vendor output locked in d4 source': 'M2.4d4 vendor IO LOCKED' in (base/'m24d4_z1_transport_contract/include/z1_transport_contract.h').read_text(),
 'no new SDK communication in sim bridge': not any(s in (mod/'src/m24d5_sim_gate.cpp').read_text() for s in ('sendRecv(', 'startTrack(', 'recvfrom(', 'sendto(', 'socket(')),
}
for k,v in checks.items(): print(f"[{'PASS' if v else 'FAIL'}] {k}")
if not all(checks.values()):sys.exit(2)
print('M2.4d5 STATIC INTEGRATION BOUNDARY: PASS (requires real b2z1_ctrl rebuild+MuJoCo test)')
