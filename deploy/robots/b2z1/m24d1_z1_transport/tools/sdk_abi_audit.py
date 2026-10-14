#!/usr/bin/env python3
"""Static ABI/report only. Cannot establish runtime datagram proof."""
import argparse,subprocess,sys,re
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('library',type=Path)
a=p.parse_args()
lib=a.library.resolve()
if not lib.is_file(): sys.exit('Z1 SDK .so missing: '+str(lib))
print('Z1 SDK ELF audit:',lib)
for cmd in (['readelf','-h',str(lib)],['nm','-D','-C',str(lib)]):
    try:
        c=subprocess.run(cmd,capture_output=True,text=True,timeout=20)
    except (OSError,subprocess.TimeoutExpired) as e:
        print('TOOL UNAVAILABLE:',e);continue
    if cmd[0]=='readelf':
        for line in c.stdout.splitlines():
            if 'Class:' in line or 'Machine:' in line or 'Type:' in line: print(line.strip())
    else:
        lines=[x for x in c.stdout.splitlines() if re.search(r'UDPPort::recv|CtrlComponents::sendRecv|unitreeArm::(sendRecv|startTrack)|\brecvfrom\b|\brecvmsg\b|\brecv\b|\brecvmmsg\b|sendto|sendmsg',x)]
        print('Relevant dynamic symbols (truncated to 50):')
        for x in lines[:50]:print(' ',x)
        if not lines: print('  (none visible; symbols may be stripped/hidden)')
    if c.returncode:print(cmd[0],'returned',c.returncode,'(nonfatal informational audit)')
print('CONCLUSION: ELF symbols DO NOT prove fresh UDP reception, synchronization,')
print('          correct sender identity or a matching decoded SDK state.')
print('          Hardware control stays LOCKED until independently verified RX hook.')
