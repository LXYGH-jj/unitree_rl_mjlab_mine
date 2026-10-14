#!/usr/bin/env python3
from pathlib import Path
import sys
repo=Path(sys.argv[1]).resolve()
base=repo/'deploy/robots/b2z1'
main=(base/'main.cpp').read_text()
new=(base/'m24d1_z1_transport')
src='\n'.join(p.read_text() for p in sorted((new/'src').glob('*.cpp')))
head='\n'.join(p.read_text() for p in sorted((new/'include').glob('*.h')))
checks={
 'Pre-DDS hardware refusal preserved': main.index('M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED') < main.index('ChannelFactory::Instance()->Init'),
 'No vendor send/SDK thread/constructor in new module': not any(x in src for x in ('sendRecv(', 'startTrack(', 'setArmCmd(', 'sendRecvThread->start(', 'unitreeArm arm(', 'make_unique<UNITREE_ARM::unitreeArm>')),
 'SDK snapshot explicitly marked getter only': 'copySdkGetterSnapshotUnverified' in src and 'kRealVendorRxProofAvailable = false' in head,
 'Getter polling does not provide proof': 'return std::nullopt;' in src and 'fromSdkGetterOnly' in src,
 'Actual receive evidence required': 'snapshot_decoded_from_this_datagram' in src and 'completed_receive_index' in src,
 'Motor temperature interlock present': 'max_motor_temperature_c' in src,
 'Session uses the real M2.4c3 output gate': 'gate_.controlTick(now)' in src,
 'Only CMake integration source additions': 'm24d1_z1_transport/src/z1_rx_validation.cpp' in (base/'CMakeLists.txt').read_text(),
}
for k,v in checks.items():
    print(('[PASS]' if v else '[FAIL]'),k)
if not all(checks.values()):sys.exit(2)
print('M2.4d1 static boundary audit PASS (static only; no real Z1 receiver hook).')
