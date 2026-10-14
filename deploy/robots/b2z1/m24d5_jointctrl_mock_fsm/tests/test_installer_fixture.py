#!/usr/bin/env python3
"""Test deterministic patching against exact baseline anchors; no actual Git tree needed."""
from pathlib import Path
from tempfile import TemporaryDirectory
import importlib.util
import sys
from unittest.mock import patch
spec=importlib.util.spec_from_file_location('installer', Path(sys.argv[1])/'tools/install_overlay.py')
ins=importlib.util.module_from_spec(spec)
spec.loader.exec_module(ins)
from types import SimpleNamespace
with TemporaryDirectory() as td:
    repo=Path(td)
    sources={
      'deploy/robots/b2z1/main.cpp': '''#include "FSM/CtrlFSM.h"
#include "State_JointCtrlShadow.h"
M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED
    unitree::robot::ChannelFactory::Instance()->Init(0, network);
''',
      'deploy/robots/b2z1/CMakeLists.txt': '''  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/include
  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/src/z1_transport_session.cpp
''',
      'deploy/robots/b2z1/include/State_InteractiveArmIK.h': '''#include "interactive_pipeline.h"
class State_InteractiveArmIK final : public FSMState {
    bool has_walked_{false};
''',
      'deploy/robots/b2z1/src/State_InteractiveArmIK.cpp': '''#include <limits>
    : FSMState(state, std::move(state_string)) {
void State_InteractiveArmIK::clear_motion() {
}
    clear_motion();
}

void State_InteractiveArmIK::enter()
    capture_arm_goal(s);
    commanded_ = s.leg_q;
    if(!arm_motion_locked_ && !fault_) {
        for(int i=0;i<6;++i)
            arm_cmd_[i]+=std::clamp(arm_goal_q_[i]-arm_cmd_[i],-max_arm_step_rad_,max_arm_step_rad_);
    }
        if(fault_)hold_on_policy_fault();
        else motor_step(state,now);
void State_InteractiveArmIK::exit() {
    clear_motion();
''',
      'deploy/robots/b2z1/config/config_interactive_jointctrl_shadow_m24c2_sim.yaml': '''FSM:
  _:
    JointCtrlShadow:
      id: 7
      type: JointCtrlShadow
  FixStand:
    transitions:
      JointCtrlShadow: RT + B.on_pressed
  JointCtrlShadow:
    transitions:
      FixStand: LT + up.on_pressed
      Passive: LT + B.on_pressed
      SlowLieDown: LT + down.on_pressed
'''
    }
    for p,v in sources.items():
        f=repo/p; f.parent.mkdir(parents=True,exist_ok=True);f.write_text(v)
    with patch.object(ins,'git',side_effect=lambda *args: SimpleNamespace(returncode=0)):
        out=ins.content(repo)
    assert all(k in out for k in ins.TARGETS+[ins.NEW_CFG])
    assert out[ins.NEW_CFG].count('type: JointCtrlMockSim')==1
    assert 'JointCtrlShadow:\n      id:' not in out[ins.NEW_CFG]
    assert out['deploy/robots/b2z1/main.cpp'].find('M2.4d5 refuses non-loopback') < out['deploy/robots/b2z1/main.cpp'].find('ChannelFactory::Instance()->Init')
    assert 'mock_sim_gate_->step' in out['deploy/robots/b2z1/src/State_InteractiveArmIK.cpp']
    assert 'm24d5_sim_gate.cpp' in out['deploy/robots/b2z1/CMakeLists.txt']
print('M2.4d5 installer deterministic fixture: PASS')
