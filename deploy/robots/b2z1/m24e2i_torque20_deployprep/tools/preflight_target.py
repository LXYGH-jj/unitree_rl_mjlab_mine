#!/usr/bin/env python3
"""Non-actuating architecture/artifact preflight; never starts robot software."""
import argparse,hashlib,platform,subprocess,sys
from pathlib import Path

def main():
  ap=argparse.ArgumentParser()
  ap.add_argument('repo',type=Path)
  ap.add_argument('--software-only',action='store_true',help='Skip runtime libraries on this host')
  a=ap.parse_args()
  repo=a.repo.expanduser().resolve(strict=True)
  b=repo/'deploy/robots/b2z1'
  arm=platform.machine()
  expected=repo/f'deploy/thirdparty/z1_sdk/lib/libZ1_SDK_{arm}.so'
  needed={
   'torque20': b/'m24e2d_sdk_worker_bridge/include/m24e2h_gravity_guard.h',
   'z1_worker': b/'m24e2d_sdk_worker_bridge/src/z1_worker_dual.cpp',
   'planner': b/'m24e2b_z1_lowcmd_adapter/src/z1_lowcmd_plan.cpp',
   'ipc_client': b/'m24e2e_live_input/src/z1_ipc_client.cpp',
   'native_FSM': b/'m24e2g_real_control/src/State_InteractiveArmReal.cpp',
   'live_config': b/'config/config_m24e2g_real_candidate.yaml',
   'onnx_input': repo/'deploy/interactive_b2z1/src/interactive_observation.cpp',
 }
  print('HOST architecture:',arm,'OS:',platform.system())
  failed=[]
  for name,path in needed.items():
    good=path.is_file(); print(('PASS' if good else 'MISSING'),name,path)
    if not good:failed.append(name)
  h=needed['torque20']
  if h.is_file():
    v=h.read_text()
    good='kModelTorqueSanityBoundNm=20.0;' in v and 'kModelTorqueSanityBoundNm=50.0;' not in v
    print(('PASS' if good else 'FAIL'),'20Nm header content')
    if not good:failed.append('20Nm')
    print('SHA256 torque header:',hashlib.sha256(h.read_bytes()).hexdigest())
  print('Z1 SDK per-architecture binary:',expected, 'PRESENT' if expected.is_file() else 'NOT FOUND')
  if not a.software_only and not expected.is_file(): failed.append('SDK_LIB_'+arm)
  for binary in [b/'build/b2z1_ctrl',b/'build/b2z1_z1_kin_probe']:
    if binary.is_file():
      o=subprocess.run(['file',str(binary)],capture_output=True,text=True)
      print('EXISTING_BINARY:',o.stdout.strip())
      if not a.software_only:
        l=subprocess.run(['ldd',str(binary)],capture_output=True,text=True)
        for line in l.stdout.splitlines():
          if 'not found' in line:
            failed.append('ldd_'+binary.name)
            print('MISSING_LIB:',line.strip())
    else:print('INFO missing target-build artifact:',binary)
  cfg=needed['live_config']
  if cfg.is_file():
    txt=cfg.read_text()
    # Best-effort text check; this is not a YAML parser.
    for key in ['enable_b2_output: false','enable_z1_sdk_targets: false']:
      print('INFO hardware gate',key, 'FOUND' if key in txt else 'NOT FOUND (review YAML)')
  if failed:
    print('PREFLIGHT FAIL:',','.join(failed));sys.exit(1)
  print('M2.4e2i SOFTWARE PREFLIGHT PASS; NO HARDWARE I/O executed')
if __name__=='__main__': main()
