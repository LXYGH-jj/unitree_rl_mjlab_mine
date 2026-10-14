#!/usr/bin/env python3
"""Add-only M2.4c3 JOINTCTRL safety-gated command path; no B2/FSM changes."""
from __future__ import annotations
import argparse
import difflib
from datetime import datetime
from pathlib import Path
import shutil
import sys

ROOT=Path(__file__).resolve().parents[1]
MODULE=Path('deploy/robots/b2z1/m24c3_jointctrl_runtime')
CMAKE=Path('deploy/robots/b2z1/CMakeLists.txt')

INC_ANCHOR='  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/m24c_shadow_integration/include\n'
SRC_ANCHOR='  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/m24c_shadow_integration/src/m24c_shadow_bridge.cpp\n'
INC_ADD='  ${PROJECT_SOURCE_DIR}/m24b2_jointctrl_offline/include\n  ${PROJECT_SOURCE_DIR}/m24c3_jointctrl_runtime/include\n'
SRC_ADD='  ${PROJECT_SOURCE_DIR}/m24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp\n  ${PROJECT_SOURCE_DIR}/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp\n'

def proposed(cmake: str) -> str:
    for anchor in (INC_ANCHOR,SRC_ANCHOR):
        if cmake.count(anchor)!=1:
            raise RuntimeError(f'M2.4c2 CMake anchor must appear exactly once: {anchor.strip()}')
    if 'm24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp' in cmake:
        raise RuntimeError('M2.4c3 CMake changes already applied; refusing repeat')
    return cmake.replace(INC_ANCHOR,INC_ANCHOR+INC_ADD).replace(SRC_ANCHOR,SRC_ANCHOR+SRC_ADD)

def make_plan(repo: Path):
    if not (repo/'.git').exists(): raise RuntimeError(f'not a git checkout: {repo}')
    cmake=repo/CMAKE
    if not cmake.is_file(): raise FileNotFoundError(cmake)
    dependencies=(
        'deploy/robots/b2z1/m24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp',
        'deploy/robots/b2z1/m24b2_jointctrl_offline/include/z1_jointctrl_safety.h',
        'deploy/robots/b2z1/src/State_JointCtrlShadow.cpp',
        'deploy/interactive_b2z1/m24c_shadow_integration/include/m24c_shadow_bridge.h',
    )
    for path in dependencies:
        if not (repo/path).is_file(): raise FileNotFoundError('Prerequisite M2.4b2/c2 missing: '+path)
    old=cmake.read_text()
    new=proposed(old)
    if (repo/MODULE).exists():
        raise FileExistsError(f'{repo/MODULE} exists; refusing overwrite')
    return old,new

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('repo',type=Path)
    grp=parser.add_mutually_exclusive_group(required=True)
    grp.add_argument('--check',action='store_true')
    grp.add_argument('--apply',action='store_true')
    args=parser.parse_args()
    repo=args.repo.expanduser().resolve()
    old,new=make_plan(repo)
    if args.check:
        print(''.join(difflib.unified_diff(old.splitlines(True),new.splitlines(True),
             fromfile='CMakeLists.txt.before',tofile='CMakeLists.txt.M24c3')))
        print('NEW MODULE:',MODULE)
        print('No real SDK send or B2 motor path modifications; no files changed.')
        return
    backup=Path.home()/'lxy'/'b2z1_backups'/('M24c3_pre_install_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    backup.mkdir(parents=True,exist_ok=False)
    shutil.copy2(repo/CMAKE,backup/'CMakeLists.txt')
    try:
        shutil.copytree(ROOT,repo/MODULE,ignore=shutil.ignore_patterns('__pycache__','*.pyc','build'))
        (repo/CMAKE).write_text(new)
    except Exception:
        (repo/CMAKE).write_text(old)
        if (repo/MODULE).exists(): shutil.rmtree(repo/MODULE)
        raise
    print('M2.4c3 JOINTCTRL software execution gate INSTALLED: '+str(repo/MODULE))
    print('Backed up original CMake to '+str(backup))
    print('Only CMake+new module changed; SDK/B2/FSM/M2.3/M2.4c2 sources untouched.')
    print('NO real vendor transport or motor I/O added; this is NOT hardware permission.')

if __name__=='__main__':
    try: main()
    except Exception as e:
        print('[M24c3 FAIL CLOSED]',str(e),file=sys.stderr)
        sys.exit(2)
