#!/usr/bin/env python3
"""Static invariants only: don't misrepresent as proof of hardware behavior."""
from pathlib import Path
import sys
root=Path(__file__).resolve().parent.parent
s=(root/'src/State_JointCtrlShadow.cpp').read_text()
h=(root/'include/State_JointCtrlShadow.h').read_text()
i=(root/'tools/install_overlay.py').read_text()
checks={
    'SIM-only env ACK gate': 'B2Z1_SIM_ONLY_ACK' in s and 'MUJOCO_LOOPBACK_NO_HARDWARE' in s,
    'Forbids legacy LOWCMD SDK': 'M2.4c2 forbids legacy LOWCMD SDK bridge' in s,
    'Forbids vendor JOINTCTRL output': 'M2.4c2 forbids real Z1 SDK JOINTCTRL output' in s,
    'Bridge uses true 799D pipeline': 'ShadowBridge' in h and 'shadow_->preview' in s,
    'No vendor SDK sendRecv in new state': all(x not in s for x in ('sendRecv(', 'setArmCmd(', 'startTrack(', 'setFsm(', 'backToStart(')),
    'No actor-output motor writes': 'p.leg_policy_action[0]' in s and 'motor.q(arm_hold_' in s and 'motors[i].q(leg_hold_' in s,
    'Sim synthetic-frame proof documented': 'one strictly advancing MuJoCo synthetic tick' in s,
    'Main guard BEFORE B2 DDS Init': i.find('const auto jointctrl_hardware') >= 0 and i.find('gate + anchor') >= 0 and 'const auto jointctrl_hardware' in i.split('gate =')[1],
    'No direct changes to old LOWCMD source': 'z1_sdk_controller.cpp' not in i,
}
for k,v in checks.items():print(('[PASS]' if v else '[FAIL]')+' '+k)
if not all(checks.values()):sys.exit(2)
print('STATIC AUDIT ONLY; full b2z1_ctrl build in developer checkout is REQUIRED.')
