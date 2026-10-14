#!/usr/bin/env python3
"""M2.4d1: install offline-only SDK feedback proof boundary, with no FSM/B2 changes."""
from __future__ import annotations
import argparse
import difflib
from datetime import datetime
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
MODULE=Path('deploy/robots/b2z1/m24d1_z1_transport')
CMAKE=Path('deploy/robots/b2z1/CMakeLists.txt')
INC_ANCHOR='  ${PROJECT_SOURCE_DIR}/m24c3_jointctrl_runtime/include\n'
SRC_ANCHOR='  ${PROJECT_SOURCE_DIR}/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp\n'
INC_ADD='  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/include\n'
SRC_ADD='  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/src/z1_rx_validation.cpp\n  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/src/z1_transport_session.cpp\n  ${PROJECT_SOURCE_DIR}/m24d1_z1_transport/src/z1_sdk_getter_snapshot.cpp\n'

def proposed(old: str) -> str:
    for anchor in (INC_ANCHOR,SRC_ANCHOR):
        if old.count(anchor)!=1:
            raise RuntimeError('Expected M2.4c3 anchor exactly once: '+anchor.strip())
    if 'm24d1_z1_transport/src/z1_rx_validation.cpp' in old:
        raise RuntimeError('M2.4d1 already installed; refusing duplicate')
    return old.replace(INC_ANCHOR,INC_ANCHOR+INC_ADD).replace(SRC_ANCHOR,SRC_ANCHOR+SRC_ADD)

def plan(repo: Path):
    if not (repo/'.git').exists(): raise RuntimeError('Not Git working tree: '+str(repo))
    needs=[
        'deploy/robots/b2z1/m24b2_jointctrl_offline/include/z1_jointctrl_safety.h',
        'deploy/robots/b2z1/m24c3_jointctrl_runtime/include/z1_jointctrl_session.h',
        'deploy/robots/b2z1/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp',
        'deploy/thirdparty/z1_sdk/include/unitree_arm_sdk/control/unitreeArm.h',
    ]
    for x in needs:
        if not (repo/x).is_file(): raise FileNotFoundError('Missing prerequisite: '+x)
    if (repo/MODULE).exists(): raise FileExistsError('Module already exists; refusing overwrite')
    old=(repo/CMAKE).read_text();return old,proposed(old)

def main():
    p=argparse.ArgumentParser()
    p.add_argument('repo',type=Path)
    group=p.add_mutually_exclusive_group(required=True)
    group.add_argument('--check',action='store_true')
    group.add_argument('--apply',action='store_true')
    args=p.parse_args()
    repo=args.repo.expanduser().resolve()
    old,new=plan(repo)
    if args.check:
        print(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile='CMake.before',tofile='CMake.M24d1')))
        print('New module:',MODULE)
        print('Existing B2/FSM/SDK code left untouched. Real vendor IO remains locked.')
        return
    backup=Path.home()/'lxy'/'b2z1_backups'/('M24d1_pre_install_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True,exist_ok=False)
    shutil.copy2(repo/CMAKE,backup/'CMakeLists.txt')
    try:
        shutil.copytree(ROOT,repo/MODULE,ignore=shutil.ignore_patterns('__pycache__','*.pyc','build'))
        (repo/CMAKE).write_text(new)
    except Exception:
        (repo/CMAKE).write_text(old)
        if (repo/MODULE).exists():shutil.rmtree(repo/MODULE)
        raise
    print('M2.4d1 installed:',repo/MODULE)
    print('CMake original backed up:',backup)
    print('NO SDK construction/UDP/transmit, NO FSM or legacy bridge modification.')

if __name__=='__main__':
    try:main()
    except Exception as e:
        print('[M24d1 FAIL CLOSED]',e,file=sys.stderr)
        sys.exit(2)
