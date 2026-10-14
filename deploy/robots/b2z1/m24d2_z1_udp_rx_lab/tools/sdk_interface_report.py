#!/usr/bin/env python3
"""Read-only vendor-header capability report. This does NOT touch a robot or SDK .so."""
from pathlib import Path
import re
import sys
repo=Path(sys.argv[1]).resolve()
root=repo/'deploy/thirdparty/z1_sdk/include/unitree_arm_sdk'
paths={
  'wire':root/'message/arm_common.h',
  'udp':root/'message/udp.h',
  'components':root/'control/ctrlComponents.h',
  'arm':root/'control/unitreeArm.h',
}
if not all(p.is_file() for p in paths.values()):
    print('SDK header report: public headers missing in fixture; real checkout should be audited.')
    sys.exit(0)
files={k:p.read_text() for k,p in paths.items()}
recv_match=re.search(r'\bstruct\s+RecvState\s*\{([^}]*)\}',files['wire'],re.DOTALL)
print('===== M2.4d2 vendor public header report (read-only) =====')
if recv_match:
    body=recv_match.group(1)
    print('RecvState public fields:')
    for ln in body.splitlines():
        if ln.strip() and not ln.strip().startswith('//'):
            print(' ',ln.strip())
    print('Explicit receive sequence/timestamp in public RecvState: ',
          bool(re.search(r'\b(seq(?:uence)?|timestamp|recv_time|frame_id)\b',body,re.I)))
else:
    print('RecvState was not found in public header; no assumption about wire format.')
print('UDPPort::recv publicly declared:', 'recv(uint8_t *recvMsg' in files['udp'])
print('unitreeArm::sendRecv publicly declared:', 'void sendRecv()' in files['arm'])
print('Unitree SDK receive callback advertised in provided headers:',
      'onReceive' in files['arm'] or 'setReceiveCallback' in files['arm'])
print('CONCLUSION: public API and ELF declarations alone do NOT demonstrate')
print('receiver-origin peer validation, atomic decoded state or freshness proof.')
print('Do not use this report or mock loopback CRC/sequence as real SDK evidence.')
