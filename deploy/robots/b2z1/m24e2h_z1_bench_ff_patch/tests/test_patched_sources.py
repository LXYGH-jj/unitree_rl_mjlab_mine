#!/usr/bin/env python3
import sys
from pathlib import Path
r=Path(sys.argv[1])
w=(r/'src/z1_worker_dual.cpp').read_text()
s=(r/'src/z1_sdk_bridge.cpp').read_text()
assert 'SdkBridge>(has_gripper)' in w
assert 'SDK needs explicit --has-gripper yes|no' in w
assert 'session(client.fd,sdk_mode,has_gripper)' in w
assert s.count('inverseDynamics(')==2
assert s.count('m24e2h::combineGravityTorque(')==2
assert (r/'include/m24e2h_gravity_guard.h').is_file()
print('M2.4e2h source guards PASS')
