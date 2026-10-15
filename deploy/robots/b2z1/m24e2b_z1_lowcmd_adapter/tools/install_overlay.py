#!/usr/bin/env python3
"""Add-only installer; no edits to working B2, FSM or prior e2a."""
import argparse
from pathlib import Path
import shutil
import subprocess

def git(repo,*args):return subprocess.check_output(['git','-C',str(repo),*args],text=True).strip()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('repo',type=Path)
    m=ap.add_mutually_exclusive_group(required=True);m.add_argument('--check',action='store_true');m.add_argument('--apply',action='store_true')
    a=ap.parse_args(); repo=a.repo.resolve(strict=True)
    target=repo/'deploy/robots/b2z1/m24e2b_z1_lowcmd_adapter'
    src=Path(__file__).resolve().parents[1]
    assert (repo/'deploy/robots/b2z1/m24e2a_z1_worker_ipc/src/z1_worker_mock.cpp').is_file(), 'M2.4e2a must be installed first'
    assert git(repo,'branch','--show-current') in ('feature/b2z1-interactive-deploy','feature/b2z1-alore-real-deploy'), 'wrong branch'
    assert subprocess.run(['git','-C',str(repo),'merge-base','--is-ancestor','f2a3567','HEAD']).returncode==0, 'base f2a3567 not present'
    assert not target.exists() and not target.is_symlink(), f'target exists {target}'
    print('M2.4e2b ADD ONLY:',target)
    print('No changes to B2/FSM/ONNX/MuJoCo/IPC V1, or Z1 SDK files.')
    if a.check:print('DRY RUN: no changes')
    else:
        shutil.copytree(src,target,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
        print('INSTALLED:',target)
if __name__=='__main__':main()
