#!/usr/bin/env python3
from pathlib import Path
import sys
base=Path(__file__).resolve().parents[1]
repo=Path(sys.argv[1]).resolve()
src=(base/'src/z1_jointctrl_session.cpp').read_text()
hdr=(base/'include/z1_jointctrl_session.h').read_text()
main=(repo/'deploy/robots/b2z1/main.cpp').read_text()
cmake=(repo/'deploy/robots/b2z1/CMakeLists.txt').read_text()
checks={
'Z1 transport is dependency-injected': 'CommandSink& sink_' in hdr,
'No SDK/DDS/motor call in execution gate': not any(k in src for k in ('sendRecv','startTrack','setArmCmd','LowCmd','ChannelFactory','motor_cmd')),
'No real socket or SDK header in execution gate': not any(k in hdr for k in ('unitreeArm','unitree_go','unitree_sdk2')),
'Pre-DDS real JOINTCTRL refusal preserved': main.find('REAL Z1 JOINTCTRL hardware path LOCKED')>=0 and main.find('REAL Z1 JOINTCTRL hardware path LOCKED') < main.find('ChannelFactory::Instance()->Init'),
'Current main retains legacy B2 path': 'release_mode_before_start' in main,
'Gate included in formal CMake target': 'm24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp' in cmake,
'Existing safety core built into target': 'm24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp' in cmake,
}
for label,ok in checks.items():
    print(('[PASS] ' if ok else '[FAIL] ')+label)
if not all(checks.values()): sys.exit(2)
print('STATIC AUDIT ONLY. No Z1 JOINTCTRL bench approval or real UDP packet proof exists.')
