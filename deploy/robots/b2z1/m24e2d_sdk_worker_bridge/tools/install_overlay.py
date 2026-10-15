#!/usr/bin/env python3
"""M2.4e2d add-only installer; no production C++/SDK/B2 modifications."""
import argparse
import shutil
import subprocess
from pathlib import Path

def git(repo, *args):
    return subprocess.check_output(['git','-C',str(repo),*args],text=True).strip()

def main():
    p=argparse.ArgumentParser()
    p.add_argument('repo',type=Path)
    m=p.add_mutually_exclusive_group(required=True)
    m.add_argument('--check',action='store_true')
    m.add_argument('--apply',action='store_true')
    args=p.parse_args()
    repo=args.repo.resolve(strict=True)
    root=repo/'deploy/robots/b2z1'
    src=Path(__file__).resolve().parents[1]
    dest=root/'m24e2d_sdk_worker_bridge'
    branch=git(repo,'branch','--show-current')
    if branch not in ('feature/b2z1-interactive-deploy','feature/b2z1-alore-real-deploy'):
        raise RuntimeError('Refusing unexpected branch: '+branch)
    subprocess.run(['git','-C',str(repo),'merge-base','--is-ancestor','f2a3567','HEAD'],check=True)
    prereq=[
        'm24e2a_z1_worker_ipc/tests/test_ipc.py',
        'm24e2b_z1_lowcmd_adapter/include/z1_lowcmd_plan.h',
        'm24e2b_z1_lowcmd_adapter/src/z1_lowcmd_plan.cpp',
        'm24e2c_ipc_planner_integration/tests/test_integration.py',
    ]
    for path in prereq:
        if not (root/path).is_file(): raise RuntimeError('Missing prior overlay: '+path)
    if dest.exists() or dest.is_symlink(): raise RuntimeError('Destination already exists; refusing overwrite: '+str(dest))
    for path in ('src/z1_worker_dual.cpp','src/z1_sdk_bridge.cpp','include/z1_sdk_bridge.h','CMakeLists.txt','run_offline_tests.sh','run_sdk_linkcheck.sh'):
        if not (src/path).is_file(): raise RuntimeError('Missing overlay file: '+path)
    print('BASE f2a3567 ancestor verified; branch='+branch)
    print('M2.4e2a/e2b/e2c found; ADD ONLY '+str(dest))
    print('No production B2/FSM/ONNX/SDK or parent CMake modifications. Default build OFF (no physical SDK).')
    if args.check:print('DRY RUN: no changes')
    else:
        shutil.copytree(src,dest,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        print('INSTALLED:',dest)

if __name__=='__main__':main()
