#!/usr/bin/env python3
"""Install E2f preview-only FSM; change main/CMake only with exact anchors and backup."""
import argparse
import pathlib
import shutil
import subprocess
import sys
import os
import textwrap

SRC=pathlib.Path(__file__).resolve().parents[1]
SUBDIR=pathlib.Path('deploy/robots/b2z1/m24e2f_real_fsm_preview')
MAIN=pathlib.Path('deploy/robots/b2z1/main.cpp')
CMAKE=pathlib.Path('deploy/robots/b2z1/CMakeLists.txt')
CONFBASE=pathlib.Path('deploy/robots/b2z1/config/config_interactive_arm_ik_sim.yaml')
CONFNEW=pathlib.Path('deploy/robots/b2z1/config/config_m24e2f_real_input_preview_sim.yaml')

HEADER_ANCHOR='#include "State_JointCtrlShadow.h"'
HEADER_ADD='#include "State_InteractiveArmRealPreview.h" // M2.4e2f preview ONLY'
GUARD_ANCHOR='    unitree::robot::ChannelFactory::Instance()->Init(0, network);'
GUARD_ADD='''    // M2.4e2f is not a real-hardware FSM. Fail before DDS or mode release.
    if (param::config["interactive_real_preview"] && !loopback_network) {
        spdlog::critical("M2.4e2f preview refuses non-loopback network before DDS");
        return 2;
    }
'''
INCL_ANCHOR='include_directories(\n'
INCL_ADD='''include_directories(
  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/include
  ${PROJECT_SOURCE_DIR}/m24e2e_live_input/include
'''
SOURCE_ANCHOR='target_compile_definitions(${PROJECT_NAME}_lib PUBLIC'
SOURCE_ADD='''# M2.4e2f: controlled SIM-ONLY FSM preview; does not send new motor targets.
target_sources(${PROJECT_NAME}_lib PRIVATE
  ${PROJECT_SOURCE_DIR}/m24e2f_real_fsm_preview/src/State_InteractiveArmRealPreview.cpp
  ${PROJECT_SOURCE_DIR}/m24e2e_live_input/src/z1_ipc_client.cpp
  ${PROJECT_SOURCE_DIR}/m24e2e_live_input/src/b2z1_live_input.cpp
)
'''

def assert_valid(repo):
    for f in [MAIN,CMAKE,CONFBASE]:
        if not (repo/f).is_file():raise RuntimeError(f'missing {f}')
    deps=['m24e2a_z1_worker_ipc','m24e2b_z1_lowcmd_adapter','m24e2d_sdk_worker_bridge','m24e2e_live_input']
    for dep in deps:
        if not (repo/'deploy/robots/b2z1'/dep).is_dir():raise RuntimeError('missing dependency: '+dep)
    check=subprocess.run(['git','-C',str(repo),'merge-base','--is-ancestor','f2a3567','HEAD'],capture_output=True)
    if check.returncode:raise RuntimeError('baseline f2a3567 is not ancestor of current repo')

def transform(main,cmake):
    if HEADER_ADD not in main:
        if main.count(HEADER_ANCHOR)!=1:raise RuntimeError('main include anchor mismatch')
        main=main.replace(HEADER_ANCHOR,HEADER_ANCHOR+'\n'+HEADER_ADD,1)
    if GUARD_ADD not in main:
        if main.count(GUARD_ANCHOR)!=1:raise RuntimeError('main DDS anchor mismatch')
        main=main.replace(GUARD_ANCHOR,GUARD_ADD+GUARD_ANCHOR,1)
    if INCL_ADD not in cmake:
        if cmake.count(INCL_ANCHOR)!=1:raise RuntimeError('CMake include anchor mismatch')
        cmake=cmake.replace(INCL_ANCHOR,INCL_ADD,1)
    if SOURCE_ADD not in cmake:
        if cmake.count(SOURCE_ANCHOR)!=1:raise RuntimeError('CMake source anchor mismatch')
        cmake=cmake.replace(SOURCE_ANCHOR,SOURCE_ADD+SOURCE_ANCHOR,1)
    return main,cmake

def conf_transform(content):
    import yaml
    c=yaml.safe_load(content)
    if not isinstance(c,dict):raise RuntimeError('bad source YAML')
    states=c['FSM']['_']
    if 'InteractiveArmRealPreview' in states:raise RuntimeError('preview state already in base')
    used={int(v['id']) for v in states.values()}
    state_id=next(n for n in range(6,100) if n not in used)
    states['InteractiveArmRealPreview']={'id':state_id, 'type':'InteractiveArmRealPreview'}
    fix=c['FSM']['FixStand']['transitions']
    if 'InteractiveArmRealPreview' in fix:raise RuntimeError('duplicate transition')
    fix['InteractiveArmRealPreview']='RT + Y.on_pressed'
    c['FSM']['InteractiveArmRealPreview']={'transitions':{
       'FixStand':'LT + up.on_pressed', 'Passive':'LT + B.on_pressed'}}
    c['z1_sdk']['enabled']=False
    c['z1_diagnostic']['enabled']=False
    c['interactive_real_preview']={
       'sim_only':True,'backend':'mock','socket_path':'/tmp/m24e2f_z1_mock.sock',
       'policy_path': c['interactive'].get('policy_path','../../interactive_b2z1/build/b2z1_interactive_policy.onnx'),
       'ee_obs_source':'baseline_fixed','enable_ik_preview_keys':True,
       'mount_xyz_base':[0.03,0.,0.08],'tcp_offset_sdk_x':0.0882,
    }
    return '# M2.4e2f: PREVIEW ONLY. NEVER real B2 or Z1 output.\n'+yaml.safe_dump(c,allow_unicode=True,sort_keys=False)

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('repo',type=pathlib.Path)
    m=ap.add_mutually_exclusive_group(required=True)
    m.add_argument('--check',action='store_true')
    m.add_argument('--apply',action='store_true')
    args=ap.parse_args()
    repo=args.repo.resolve()
    assert_valid(repo)
    dest=repo/SUBDIR
    txt_main=(repo/MAIN).read_text();txt_cmake=(repo/CMAKE).read_text()
    newmain,newcmake=transform(txt_main,txt_cmake)
    # check parser availability now, before any changes
    config=conf_transform((repo/CONFBASE).read_text())
    if dest.exists():raise RuntimeError('refuse to overwrite existing E2f overlay')
    if (repo/CONFNEW).exists():raise RuntimeError('refuse to overwrite existing E2f YAML')
    print('E2f PREVIEW ONLY; no new B2/Z1 motor goals, no SDK loads')
    print(f'ADD {dest}; ADD {repo/CONFNEW}')
    print('PATCH native main include + pre-DDS nonloopback gate; CMake includes and sources')
    print('Original main/CMake backed up under E2f tools/backups; existing Z1 SDK remains disabled in preview config')
    if args.check:print('DRY RUN: no changes');return
    # Copy before modifying production files. If anything fails, restore from backup.
    shutil.copytree(SRC,dest,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    b=dest/'tools'/'backups';b.mkdir(exist_ok=True)
    (b/'main.cpp.before_e2f').write_text(txt_main)
    (b/'CMakeLists.txt.before_e2f').write_text(txt_cmake)
    try:
        (repo/MAIN).write_text(newmain)
        (repo/CMAKE).write_text(newcmake)
        (repo/CONFNEW).write_text(config)
    except BaseException:
        (repo/MAIN).write_text(txt_main);(repo/CMAKE).write_text(txt_cmake)
        if (repo/CONFNEW).exists():(repo/CONFNEW).unlink()
        raise
    print('INSTALLED',dest)
    print('SIM ONLY: never run this preview config on hardware. SDK target path still blocked.')
if __name__=='__main__':
    try:main()
    except Exception as e:print('INSTALL ERROR:',e,file=sys.stderr);sys.exit(2)
