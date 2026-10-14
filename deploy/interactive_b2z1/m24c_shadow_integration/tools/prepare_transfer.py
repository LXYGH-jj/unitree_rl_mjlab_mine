#!/usr/bin/env python3
"""Explicitly curate source files for transfer. Never stages unknown backup/build paths."""
import argparse
import hashlib
from pathlib import Path
import re
import shutil
import subprocess
import sys

CONFIGS = [
    'config_interactive_arm_ik_m21_sim.yaml',
    'config_interactive_arm_ik_m22a_walk_hold_sim.yaml',
    'config_interactive_arm_ik_m22b_walk_ik_sim.yaml',
    'config_interactive_arm_ik_m23_ee_goal_sim.yaml',
    'config_interactive_sim_z1_train_pose_m20c6.yaml',
]

def git(root: Path, *args, check=True):
    return subprocess.run(['git', '-C', str(root), *args], text=True,
                          capture_output=True, check=check)

def h256(p):
    digest=hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):digest.update(block)
    return digest.hexdigest()

def rel(root,p):return str(p.relative_to(root))

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('repo',type=Path)
    ap.add_argument('--prepare-model',action='store_true',help='Copy exact M2.3 ONNX into versioned models/ + create separate portable YAML. Do NOT overwrite original config.')
    ap.add_argument('--stage',action='store_true',help='Stage ONLY explicit portable source files; review before commit/push')
    a=ap.parse_args(); root=a.repo.resolve()
    if not (root/'deploy/robots/b2z1/CMakeLists.txt').is_file():ap.error('Not expected repository')
    branch=git(root,'branch','--show-current').stdout.strip()
    head=git(root,'rev-parse','--short','HEAD').stdout.strip()
    print('Branch:',branch,' HEAD:',head)
    if branch!='feature/b2z1-interactive-deploy':
        ap.error('Wrong branch; refuse staging')
    m23=root/'deploy/robots/b2z1/config/config_interactive_arm_ik_m23_ee_goal_sim.yaml'
    portable=root/'deploy/robots/b2z1/config/config_interactive_arm_ik_m23_portable_sim.yaml'
    source_model=None
    if m23.is_file():
        cfg=m23.read_text(encoding='utf-8')
        match=re.search(r'(?m)^\s*policy_path:\s*["\']?([^\s"\'\#]+)',cfg)
        if match:
            model_relative=match.group(1)
            source_model=(root/'deploy/robots/b2z1'/model_relative).resolve()
            print('M2.3 policy source:',source_model,'exists:',source_model.is_file())
        else:print('[BLOCKER] Missing policy_path in M2.3 config')
    else:print('[BLOCKER] M2.3 YAML is missing')
    model_target=root/'deploy/interactive_b2z1/models/b2z1_interactive_policy.onnx'
    if a.prepare_model:
        if source_model is None or not source_model.is_file():ap.error('Missing ONNX model; prepare/export on dev PC before transfer')
        if source_model == model_target:ap.error('Source and destination same, unexpected')
        if model_target.exists():
            if h256(source_model)!=h256(model_target):ap.error('Different model already at destination; refusing overwrite')
            print('Model already has matching hash; copy skipped')
        else:
            model_target.parent.mkdir(parents=True,exist_ok=True)
            shutil.copy2(source_model,model_target)
        if portable.exists():
            if 'interactive_b2z1/models/b2z1_interactive_policy.onnx' not in portable.read_text():
                ap.error('Portable config exists but unexpected; refusing overwrite')
        else:
            out=cfg[:match.start(1)]+'../../interactive_b2z1/models/b2z1_interactive_policy.onnx'+cfg[match.end(1):]
            portable.write_text('# M2.4c PORTABLE SIM; same M2.3 behavior, ONNX in versioned models/. NO HARDWARE.\n'+out,encoding='utf-8')
        print('ONNX SHA256:',h256(model_target),'size:',model_target.stat().st_size)
        if model_target.stat().st_size >= 95_000_000:
            print('[BLOCKER] ONNX near/above GitHub file limit. Configure Git LFS and confirm remote support BEFORE staging.')
    else:
        print('Model is not copied by default. Run --prepare-model when ready.')
    # CMake currently refers to local sim_closed_loop/*.cpp. It MUST become tracked.
    depdir=root/'deploy/interactive_b2z1/sim_closed_loop'
    dep=depdir/'joint_target_guard.cpp'
    if not dep.is_file():print('[BLOCKER] Missing local CMake source:',dep)
    elif git(root,'ls-files','--error-unmatch',rel(root,dep),check=False).returncode != 0:
        print('[FIX BEFORE CLONE] Required source is UNTRACKED:',rel(root,dep))
    if not (depdir/'joint_target_guard.h').is_file():
        print('[BLOCKER] Missing joint_target_guard.h; check header location')
    # Create an explicit portable list. Never git add -A or whole backup dirs.
    candidate=[]
    fixed=[
        'deploy/include/z1_sdk_controller.h',
        'deploy/robots/b2z1/src/z1_sdk_controller.cpp',
    ]
    candidate += [root/p for p in fixed]
    candidate += [root/'deploy/robots/b2z1/config'/c for c in CONFIGS]
    candidate += [portable,model_target]
    for part in ('deploy/interactive_b2z1/sim_closed_loop',):
        candidate += [p for p in (root/part).rglob('*') if p.is_file() and p.suffix in ('.cpp','.h','.hpp')]
    for part in ('deploy/robots/b2z1/m24b2_jointctrl_offline',
                 'deploy/robots/b2z1/m24b3_z1_sdk_passive_probe',
                 'deploy/interactive_b2z1/m24c_shadow_integration'):
        d=root/part
        if not d.is_dir(): print('[BLOCKER] Missing stage directory:',d);continue
        candidate += [p for p in d.rglob('*') if p.is_file() and
                      p.suffix in ('.cpp','.h','.hpp','.py','.sh','.md','.txt')
                      and '__pycache__' not in p.parts and 'build' not in p.parts]
    candidate=sorted(set(p for p in candidate if p.is_file()),key=str)
    print('\nCurated files (NO builds/backups/logs):')
    for p in candidate:print(' ',rel(root,p))
    if a.stage:
        if not m23.is_file() or not dep.is_file() or not model_target.is_file() or not portable.is_file():
            ap.error('Transfer is incomplete: missing local M2.3 config, joint guard, model, or portable config')
        if model_target.stat().st_size>=95_000_000:
            ap.error('Model is too large for ordinary Git push; configure LFS explicitly')
        for p in candidate:
            git(root,'add','--',rel(root,p))
        print('\n[STAGED] Explicit source/model files only. Nothing committed/pushed.')
        print(git(root,'diff','--cached','--stat').stdout)
        check=git(root,'diff','--cached','--check',check=False)
        if check.returncode: print('[BLOCKER] git diff --cached --check:',check.stdout,check.stderr)
    print('\nUntracked/modified remaining items (not automatically staged):')
    print(git(root,'status','--short').stdout[:10000])
    print('Always inspect staged diff, then git commit and push manually.')

if __name__=='__main__':main()
