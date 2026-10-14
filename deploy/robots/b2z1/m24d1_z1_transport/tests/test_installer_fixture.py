#!/usr/bin/env python3
import importlib.util,tempfile,subprocess
from pathlib import Path
script=Path(__file__).resolve().parents[1]/'tools'/'install_overlay.py'
spec=importlib.util.spec_from_file_location('m24d1_installer',script)
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
cmake='A\n'+m.INC_ANCHOR+'B\n'+m.SRC_ANCHOR+'C\n'
new=m.proposed(cmake)
assert new.count('m24d1_z1_transport/src/z1_rx_validation.cpp')==1
try: m.proposed(new)
except RuntimeError: pass
else: raise AssertionError('repeat install must be rejected')
try:m.proposed('invalid CMake')
except RuntimeError: pass
else:raise AssertionError('missing anchor must be rejected')
with tempfile.TemporaryDirectory(prefix='m24d1_fixture_') as t:
    repo=Path(t);(repo/'.git').mkdir()
    for x in ['deploy/robots/b2z1/m24b2_jointctrl_offline/include/z1_jointctrl_safety.h',
              'deploy/robots/b2z1/m24c3_jointctrl_runtime/include/z1_jointctrl_session.h',
              'deploy/robots/b2z1/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp',
              'deploy/thirdparty/z1_sdk/include/unitree_arm_sdk/control/unitreeArm.h']:
        f=repo/x;f.parent.mkdir(parents=True,exist_ok=True);f.write_text('fixture')
    f=repo/m.CMAKE;f.parent.mkdir(parents=True,exist_ok=True);f.write_text(cmake)
    a=subprocess.run(['python3',str(script),str(repo),'--check'],capture_output=True,text=True)
    assert a.returncode==0 and f.read_text()==cmake,a.stderr
print('M2.4d1 installer fixture PASS (diff, repeat guard, no-modification dry run)')
