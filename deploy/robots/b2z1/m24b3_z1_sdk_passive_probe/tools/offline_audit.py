#!/usr/bin/env python3
"""Read-only audit of vendor SDK monitor source (no device operations)."""
from pathlib import Path
import sys
base = Path(sys.argv[1]).resolve()
s = (base/'src/z1_sdk_passive_probe.cpp').read_text()
checks = {
 'no motor mode switch': not any(t in s for t in ('arm.setFsm(', 'arm.startTrack(')),
 'no commanded joint target': 'arm.setArmCmd(' not in s,
 'no backToStart': 'arm.backToStart(' not in s,
 'no vendor background thread start': 'arm.sendRecvThread->start(' not in s,
 'explicit hardware ACK': 'B2Z1_Z1_PASSIVE_PROBE_ACK' in s,
 'single SDK sendRecv callsite': s.count('arm.sendRecv();') == 1,
 'no fabricated fresh packet proof': 's.independently_confirmed_new_packet=false' in s,
 'bounded telemetry collection': 'constexpr int n=20;' in s,
}
for name,ok in checks.items():
    print(('[PASS] ' if ok else '[FAIL] ')+name)
if not all(checks.values()): sys.exit(1)
print('STATIC AUDIT ONLY; sendRecv still transmits and is NOT physically receive-only.')
