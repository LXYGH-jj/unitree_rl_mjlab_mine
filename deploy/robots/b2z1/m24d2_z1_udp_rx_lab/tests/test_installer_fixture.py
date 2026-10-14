#!/usr/bin/env python3
from pathlib import Path
import subprocess
import tempfile

ROOT=Path(__file__).resolve().parents[1]
MODULE='deploy/robots/b2z1/m24d2_z1_udp_rx_lab'
PREREQS=[
 'deploy/robots/b2z1/m24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp',
 'deploy/robots/b2z1/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp',
 'deploy/robots/b2z1/m24d1_z1_transport/src/z1_transport_session.cpp',
 'deploy/robots/b2z1/m24d1_z1_transport/include/z1_rx_validation.h',
]
with tempfile.TemporaryDirectory(prefix='m24d2_installer_') as dirname:
    r=Path(dirname)
    (r/'.git').mkdir()
    for p in PREREQS:
        f=r/p;f.parent.mkdir(parents=True,exist_ok=True);f.write_text('// fixture\n')
    cmake=r/'deploy/robots/b2z1/CMakeLists.txt';cmake.parent.mkdir(parents=True,exist_ok=True)
    cmake.write_text('m24d1_z1_transport/src/z1_rx_validation.cpp\n')
    main=r/'deploy/robots/b2z1/main.cpp'
    main.write_text('M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED\nChannelFactory::Instance()->Init\n')
    before={cmake:cmake.read_bytes(),main:main.read_bytes()}
    cmd=['python3',str(ROOT/'tools/install_overlay.py'),str(r)]
    check=subprocess.run(cmd+['--check'],capture_output=True,text=True,timeout=8)
    assert check.returncode==0,check.stderr
    assert not (r/MODULE).exists()
    apply=subprocess.run(cmd+['--apply'],capture_output=True,text=True,timeout=8)
    assert apply.returncode==0,apply.stderr
    assert (r/MODULE/'src/z1_loopback_rx_lab.cpp').exists()
    assert all(x.read_bytes()==old for x,old in before.items())
    again=subprocess.run(cmd+['--apply'],capture_output=True,text=True,timeout=8)
    assert again.returncode!=0,'repeat must fail closed'
    assert all(x.read_bytes()==old for x,old in before.items())
print('M2.4d2 isolated installer/dry-run/repeat-guard: PASS')
