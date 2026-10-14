#!/usr/bin/env python3
"""Install isolated M2.4d2 loopback lab; intentionally NO formal-controller edits."""
from __future__ import annotations
import argparse
from pathlib import Path
import shutil
import sys

ROOT = Path(__file__).resolve().parents[1]
REL = Path('deploy/robots/b2z1/m24d2_z1_udp_rx_lab')
REQUIRED = [
    'deploy/robots/b2z1/CMakeLists.txt',
    'deploy/robots/b2z1/main.cpp',
    'deploy/robots/b2z1/m24b2_jointctrl_offline/src/z1_jointctrl_safety.cpp',
    'deploy/robots/b2z1/m24c3_jointctrl_runtime/src/z1_jointctrl_session.cpp',
    'deploy/robots/b2z1/m24d1_z1_transport/src/z1_transport_session.cpp',
    'deploy/robots/b2z1/m24d1_z1_transport/include/z1_rx_validation.h',
]

def validate(repo: Path):
    if not (repo/'.git').exists():
        raise RuntimeError(f'Not a git working tree: {repo}')
    for p in REQUIRED:
        if not (repo/p).is_file():
            raise FileNotFoundError(f'Missing prerequisite: {p}')
    if (repo/REL).exists():
        raise FileExistsError(f'M2.4d2 already installed at {repo/REL}')
    cmake = (repo/REQUIRED[0]).read_text()
    if 'm24d1_z1_transport/src/z1_rx_validation.cpp' not in cmake:
        raise RuntimeError('M2.4d1 formal controller integration not found')
    main = (repo/REQUIRED[1]).read_text()
    if ('M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED' not in main or
        'ChannelFactory::Instance()->Init' not in main or
        main.index('M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED') >
        main.index('ChannelFactory::Instance()->Init')):
        raise RuntimeError('Required pre-DDS hardware lock absent or reordered')

def main():
    p=argparse.ArgumentParser()
    p.add_argument('repo',type=Path)
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--check',action='store_true')
    g.add_argument('--apply',action='store_true')
    args=p.parse_args()
    repo=args.repo.expanduser().resolve()
    validate(repo)
    if args.check:
        print('M2.4d2 new module:', repo/REL)
        print('UNCHANGED: existing CMakeLists.txt, main.cpp, FSM, B2 SDK2, Z1 SDK, M2.3.')
        print('STANDALONE TEST: synthetic 127.0.0.1 UDP only; no physical motor sink.')
        return
    shutil.copytree(ROOT,repo/REL,ignore=shutil.ignore_patterns('build','__pycache__','*.pyc','.DS_Store'))
    print('M2.4d2 isolated loopback lab INSTALLED:',repo/REL)
    print('NO existing project file modified; production b2z1_ctrl binary unchanged.')
    print('REAL Z1 SDK RX proof/command path remains LOCKED.')

if __name__=='__main__':
    try:main()
    except Exception as e:
        print('[M2.4d2 FAIL CLOSED]',e,file=sys.stderr)
        sys.exit(2)
