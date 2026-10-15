#!/usr/bin/env python3
"""Add-only M2.4e2e live input adapter installer. No existing production changes."""
import argparse
import pathlib
import shutil
import subprocess

HERE = pathlib.Path(__file__).resolve().parents[1]
DEST = pathlib.Path('deploy/robots/b2z1/m24e2e_live_input')


def git(repo, *args):
    return subprocess.run(['git', '-C', str(repo), *args], check=True,
                          capture_output=True, text=True).stdout.strip()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('repo', type=pathlib.Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--check', action='store_true')
    mode.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    repo = args.repo.expanduser().resolve()
    if not (repo / 'deploy/robots/b2z1/CMakeLists.txt').is_file():
        raise SystemExit('Not a B2Z1 deployment repository')
    branch = git(repo, 'branch', '--show-current')
    if branch != 'feature/b2z1-interactive-deploy' and not branch.startswith('feature/b2z1-alore'):
        raise SystemExit(f'Unexpected branch {branch!r}: refuse')
    test = subprocess.run(['git', '-C', str(repo), 'merge-base', '--is-ancestor', 'f2a3567', 'HEAD'])
    if test.returncode:
        raise SystemExit('Missing verified base f2a3567 ancestor')
    for dep in ('m24e2a_z1_worker_ipc', 'm24e2b_z1_lowcmd_adapter',
                'm24e2c_ipc_planner_integration', 'm24e2d_sdk_worker_bridge'):
        if not (repo/'deploy/robots/b2z1'/dep).is_dir():
            raise SystemExit(f'Missing dependency: {dep}')
    target = repo/DEST
    print(f'BRANCH {branch}; base f2a3567 verified')
    print(f'ADD ONLY {target}')
    print('No modifications to B2 FSM, DDS, vendor SDK, ONNX, MuJoCo, config or hardware gates')
    if target.exists():
        raise SystemExit(f'Destination exists, no overwrite: {target}')
    if args.check:
        print('DRY RUN: unchanged')
        return
    shutil.copytree(HERE, target,
                    ignore=shutil.ignore_patterns('*.zip', '__pycache__', '.pytest_cache'))
    print('INSTALLED', target)

if __name__ == '__main__':
    main()
