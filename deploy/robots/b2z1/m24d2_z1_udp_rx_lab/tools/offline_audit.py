#!/usr/bin/env python3
"""Static confirmation only; not a replacement for compiler/integration tests."""
from pathlib import Path
import sys
repo=Path(sys.argv[1]).resolve()
base=repo/'deploy/robots/b2z1'
lab=base/'m24d2_z1_udp_rx_lab'
if not lab.exists():
    lab=Path(__file__).resolve().parents[1]
main=(base/'main.cpp').read_text()
formal=(base/'CMakeLists.txt').read_text()
source=(lab/'src/z1_loopback_rx_lab.cpp').read_text()
header=(lab/'include/z1_loopback_rx_lab.h').read_text()
cmake=(lab/'CMakeLists.txt').read_text()
checks={
    'Production b2z1_ctrl does not include synthetic receiver': 'm24d2_z1_udp_rx_lab' not in formal,
    'Pre-DDS real JOINTCTRL lock preserved': main.index('M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED') < main.index('ChannelFactory::Instance()->Init'),
    'SDK cannot be constructed or called from test receiver': all(x not in source+header for x in ('<unitree_arm_sdk/', 'sendRecv(', 'startTrack(', 'setArmCmd(', 'unitreeArm(')),
    'Receiver socket bound to loopback only': 'INADDR_LOOPBACK' in source and 'SOCK_DGRAM' in source,
    'Real recvmsg syscall checked': '::recvmsg(' in source and 'MSG_TRUNC' in source,
    'Receiver enforces exact synthetic peer port': 'expected_peer_port_' in source and 'ntohs(from.sin_port)' in source,
    'CRC and mock sequence checked before M2.4d1': 'crc32(b,n-4)' in source and 'p.sequence<=last_sequence_' in source,
    'M2.4d1 receives same-datagram synthetic event': 'session_.onReceiverCallback(p.snapshot,evidence,Clock::now())' in source,
    'All outputs have hardwired RecordingSink': 'RecordingSink sink_' in header and 'std::vector<b2z1::z1_jointctrl::JointCommand>' in header,
    'Standalone CMake avoids linking vendor SDK': 'libZ1_SDK' not in cmake and 'add_executable(m24d2_loopback_test' in cmake,
}
for k,v in checks.items():print(('[PASS]' if v else '[FAIL]'),k)
if not all(checks.values()):sys.exit(2)
print('M2.4d2 OFFLINE STATIC BOUNDARY: PASS (synthetic protocol cannot certify real SDK RX).')
