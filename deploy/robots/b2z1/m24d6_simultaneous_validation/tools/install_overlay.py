#!/usr/bin/env python3
"""Safely add the read-only M2.4d6 validation tool as ONE new directory.

Does not modify source code, config, CMake, motors, DDS, or vendor SDK.
"""
import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

DEST = Path('deploy/robots/b2z1/m24d6_simultaneous_validation')
FILES = ('README.md', 'run_offline_tests.sh', 'tools/analyze_sim_overlap.py',
         'tools/install_overlay.py', 'tests/test_analyze.py')


def run_git(repo, *argv):
    return subprocess.run(['git', '-C', str(repo), *argv], capture_output=True,
                          text=True, check=False)


def validate(repo):
    if not repo.is_dir() or repo.is_symlink():
        raise RuntimeError('repository directory missing or symlink')
    repo = repo.resolve(strict=True)
    result = run_git(repo, 'rev-parse', '--show-toplevel')
    if result.returncode or Path(result.stdout.strip()) != repo:
        raise RuntimeError('supplied path must be the Git repository root')
    branch = run_git(repo, 'branch', '--show-current')
    if branch.stdout.strip() != 'feature/b2z1-interactive-deploy':
        raise RuntimeError('expected feature/b2z1-interactive-deploy branch; current='+branch.stdout.strip())
    check = run_git(repo, 'merge-base', '--is-ancestor', 'a8b4f5a', 'HEAD')
    if check.returncode:
        raise RuntimeError('HEAD must be at or after the M2.4d5 commit a8b4f5a')
    dest = repo / DEST
    if dest.exists() or dest.is_symlink():
        raise RuntimeError(f'target exists, refusing overwrite: {dest}')
    if not (repo/'deploy/robots/b2z1/m24d5_jointctrl_mock_fsm').is_dir():
        raise RuntimeError('M2.4d5 component missing')
    src = Path(__file__).resolve().parents[1]
    for name in FILES:
        p = src / name
        if not p.is_file() or p.is_symlink():
            raise RuntimeError('source file missing or unsafe: '+str(p))
    return src, dest


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('repo_root', type=Path)
    option = ap.add_mutually_exclusive_group(required=True)
    option.add_argument('--check', action='store_true')
    option.add_argument('--apply', action='store_true')
    args = ap.parse_args()
    src, dest = validate(args.repo_root)
    print('M2.4d6 ADD ONLY:', dest)
    print('PRESERVED: b2z1_ctrl C++, FSM, YAML, ONNX, IK, controls, mock and real hardware locks')
    print('INPUT: text console logs ONLY; this tool has no motor/DDS/network IO')
    if args.check:
        print('M2.4d6 DRY RUN: no files modified')
        return 0
    parent = dest.parent
    with tempfile.TemporaryDirectory(prefix='.m24d6_tmp_', dir=parent) as tmp:
        staging = Path(tmp)/DEST.name
        staging.mkdir()
        for name in FILES:
            target = staging/name
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(src/name, target)
            shutil.copymode(src/name, target)
        os.rename(staging, dest)
    print('M2.4d6 INSTALLED:', dest)
    return 0

if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError) as e:
        print('M2.4d6 installer REFUSED:', e, file=sys.stderr)
        sys.exit(2)
