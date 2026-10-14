#!/usr/bin/env python3
"""Compile a throwaway fixture ELF; auditor only reads it. Never starts any robotics runtime."""
import json
import pathlib
import subprocess
import sys
import tempfile

script = pathlib.Path(__file__).resolve().parents[1] / 'tools/analyze_sdk_rx.py'
fixture = r'''
#include <cstddef>
#include <cstdint>
#include <sys/socket.h>
namespace UNITREE_ARM {
class UDPPort{public: size_t recv(unsigned char*); size_t recv(unsigned char*,size_t); size_t send(unsigned char*,size_t);};
size_t UDPPort::recv(unsigned char* x){return recv(x,8);}
size_t UDPPort::recv(unsigned char* x,size_t n){return ::recvfrom(-1,x,n,0,nullptr,nullptr);}
size_t UDPPort::send(unsigned char* x,size_t n){return ::sendto(-1,x,n,0,nullptr,0);}
class CtrlComponents{public: UDPPort* udp; void sendRecv();};
void CtrlComponents::sendRecv(){unsigned char x[8]; udp->recv(x,8);}
class unitreeArm{public: CtrlComponents* ctrl; void sendRecv(); void startTrack(int);};
void unitreeArm::sendRecv(){ctrl->sendRecv();}
void unitreeArm::startTrack(int){}
}
'''
with tempfile.TemporaryDirectory(prefix='m24d3_audit_test_') as tmp:
    root = pathlib.Path(tmp)
    (root/'fake.cpp').write_text(fixture)
    so = root/'fake_sdk.so'
    subprocess.run(['g++','-std=c++17','-shared','-fPIC','-O0',str(root/'fake.cpp'),'-o',str(so)],check=True)
    report = root/'out'
    subprocess.run([sys.executable, str(script), '--sdk', str(so), '--out', str(report)],check=True)
    o = json.loads((report/'evidence.json').read_text())
    assert o['symbols']['arm_sendrecv']['present']
    assert o['symbols']['ctrl_sendrecv']['present']
    assert o['symbols']['udp_recv_sized']['present']
    assert 'recvfrom' in o['imports']
    assert 'sendto' in o['imports']
    assert o['certifies_new_packet_proof'] is False
    assert o['permits_vendor_command_or_robot'] is False
    assert (report/'disassembly/udp_recv_sized.asm').is_file()
    wrong = subprocess.run([sys.executable,str(script),'--sdk',str(so),'--out',str(report),
                            '--expect-sha256','0'*64],capture_output=True,text=True)
    assert wrong.returncode != 0
print('M2.4d3 isolated ELF fixture + SHA guard: PASS (static only)')
