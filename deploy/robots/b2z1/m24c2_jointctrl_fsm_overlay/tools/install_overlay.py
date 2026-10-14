#!/usr/bin/env python3
"""Non-destructive M2.4c2 overlay installer; NO robot IO.

--check computes the proposed changes without writing.
--apply installs into a clean, correct local 7aa7c08-style source checkout.
Any modified existing files are preserved in an out-of-tree timestamped backup.
"""
from __future__ import annotations
import argparse
import difflib
import os
from pathlib import Path
import shutil
import sys
from datetime import datetime

ROOT = Path(__file__).resolve().parent.parent
PREVIEW_NAME = "config_interactive_jointctrl_shadow_m24c2_sim.yaml"
TARGET_MAIN = Path("deploy/robots/b2z1/main.cpp")
TARGET_CMAKE = Path("deploy/robots/b2z1/CMakeLists.txt")
TARGET_HEADER = Path("deploy/robots/b2z1/include/State_JointCtrlShadow.h")
TARGET_SOURCE = Path("deploy/robots/b2z1/src/State_JointCtrlShadow.cpp")
TARGET_CONFIG = Path("deploy/robots/b2z1/config") / PREVIEW_NAME
TARGET_README = Path("deploy/robots/b2z1/m24c2_jointctrl_fsm_overlay/README.md")
TARGET_TESTS = Path("deploy/robots/b2z1/m24c2_jointctrl_fsm_overlay")
BASE_YAML = Path("deploy/robots/b2z1/config/config_interactive_arm_ik_m23_portable_sim.yaml")


def exactly_once(x: str, old: str, where: str) -> str:
    n = x.count(old)
    if n != 1:
        raise RuntimeError(f"Refusing change to {where}: expected exactly one anchor, found {n}: {old!r}")
    return x


def build_main(src: str) -> str:
    anchor = '#include "State_InteractiveArmIK.h"\n'
    exactly_once(src, anchor, 'main.cpp')
    src = src.replace(anchor, anchor + '#include "State_JointCtrlShadow.h"  // M2.4c2 SIM-ONLY preview FSM\n')
    anchor = '    unitree::robot::ChannelFactory::Instance()->Init(0, network);\n'
    exactly_once(src, anchor, 'main.cpp')
    # This check MUST precede even DDS init and MotionSwitcher release.
    gate = '''    // M2.4c2: candidate JOINTCTRL control is NOT qualified for real robot IO.
    // Protect startup BEFORE any DDS, B2 mode release, Z1 SDK construction.
    const auto jointctrl_hardware = param::config["z1_jointctrl"];
    if (jointctrl_hardware && jointctrl_hardware["enabled"] &&
        jointctrl_hardware["enabled"].as<bool>()) {
        spdlog::critical("M2.4c2 REAL Z1 JOINTCTRL hardware path LOCKED; refusing before DDS init.");
        return 2;
    }
    const auto jointctrl_shadow = param::config["jointctrl_shadow"];
    if (jointctrl_shadow && !loopback_network) {
        spdlog::critical("M2.4c2 jointctrl_shadow permitted ONLY on MuJoCo loopback 'lo'.");
        return 2;
    }
    if (jointctrl_shadow) {
        const auto legacy = param::config["z1_sdk"];
        if (legacy && legacy["enabled"] && legacy["enabled"].as<bool>()) {
            spdlog::critical("M2.4c2 refuses legacy Z1 LOWCMD when shadow FSM is configured.");
            return 2;
        }
    }
'''
    return src.replace(anchor, gate + anchor)


def build_cmake(src: str) -> str:
    anchor = '  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/sim_closed_loop\n'
    exactly_once(src, anchor, 'CMakeLists.txt')
    src=src.replace(anchor, anchor +
        '  ${PROJECT_SOURCE_DIR}/../../interactive_b2z1/m24c_shadow_integration/include\n')
    anchor='  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/src/interactive_snapshot_store.cpp\n'
    exactly_once(src, anchor, 'CMakeLists.txt target_sources')
    src=src.replace(anchor, anchor +
        '  ${B2Z1_DEPLOY_ROOT}/interactive_b2z1/m24c_shadow_integration/src/m24c_shadow_bridge.cpp\n')
    return src


def build_yaml(src: str) -> str:
    anchor = '  Passive:\n    transitions:\n'
    exactly_once(src, anchor, 'portable sim YAML')
    entry = '''    JointCtrlShadow:
      id: 7
      type: JointCtrlShadow
'''
    src=src.replace(anchor,entry+anchor,1)
    anchor='      InteractiveArmIK: X.on_pressed\n'
    exactly_once(src,anchor,'portable sim YAML')
    src=src.replace(anchor,anchor+'      JointCtrlShadow: RT + B.on_pressed\n')
    anchor='z1_sdk:\n'
    exactly_once(src,anchor,'portable sim YAML')
    src=src.replace(anchor,'''  JointCtrlShadow:
    transitions:
      FixStand: LT + up.on_pressed
      Passive: LT + B.on_pressed
      SlowLieDown: LT + down.on_pressed
'''+anchor)
    # Intentionally no Z1 SDK or JOINTCTRL hardware enablement.
    return src+'''\n# Simulation-only FSM preview; this is NOT vendor JOINTCTRL motor control.
# 'enabled' for real JOINTCTRL must stay false. Any true value is rejected BEFORE B2 DDS init.
z1_jointctrl:
  enabled: false
  hardware_qualified: false
jointctrl_shadow:
  sim_only: true
  no_vendor_sdk_io: true
  no_actor_motor_output: true
'''


def plan(repo: Path):
    main=repo/TARGET_MAIN
    cmake=repo/TARGET_CMAKE
    yaml=repo/BASE_YAML
    for p in (main,cmake,yaml):
        if not p.is_file():
            raise FileNotFoundError(f"Required baseline file absent: {p}")
    if not (repo/"deploy/interactive_b2z1/m24c_shadow_integration/include/m24c_shadow_bridge.h").is_file():
        raise RuntimeError("M2.4c1 bridge missing: first checkout/push 7aa7c08")
    if not (repo/"deploy/interactive_b2z1/models/b2z1_interactive_policy.onnx").is_file():
        raise RuntimeError("Portable ONNX missing: first checkout/push 7aa7c08")
    generated={
        TARGET_MAIN:build_main(main.read_text()),
        TARGET_CMAKE:build_cmake(cmake.read_text()),
        TARGET_CONFIG:build_yaml(yaml.read_text()),
        TARGET_HEADER:(ROOT/"include/State_JointCtrlShadow.h").read_text(),
        TARGET_SOURCE:(ROOT/"src/State_JointCtrlShadow.cpp").read_text(),
    }
    # Preserve unit test/tool/readme as version-controlled overlay source in repo.
    for p in sorted(ROOT.rglob('*')):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != '.pyc':
            generated[TARGET_TESTS/p.relative_to(ROOT)]=p.read_bytes()
    return generated


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('repo',type=Path)
    g=ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--check',action='store_true')
    g.add_argument('--apply',action='store_true')
    args=ap.parse_args()
    repo=args.repo.expanduser().resolve()
    if not (repo/'.git').exists() and not (repo/'.git').is_file():
        raise RuntimeError(f"Not a Git working tree: {repo}")
    planned=plan(repo)
    # Preflight ALL collisions first, before doing any writes.
    for rel,new in planned.items():
        dest=repo/rel
        if rel in (TARGET_MAIN,TARGET_CMAKE):
            if isinstance(new,bytes) or dest.read_text()==new:
                raise RuntimeError(f"Already applied or invalid patch at {rel}")
        elif dest.exists():
            raise FileExistsError(f"No overwriting existing untracked path: {dest}")
    if args.check:
        for rel in (TARGET_MAIN,TARGET_CMAKE):
            old=(repo/rel).read_text().splitlines(keepends=True)
            new=planned[rel].splitlines(keepends=True)
            print(''.join(difflib.unified_diff(old,new,str(rel)+'.before',str(rel)+'.M24c2')))
        print('--- NEW FILES (not shown inline) ---')
        for rel in planned:
            if rel not in (TARGET_MAIN,TARGET_CMAKE): print(rel)
        print('CHECK ONLY: no files were changed.')
        return
    backup=Path.home()/"lxy"/"b2z1_backups"/("M24c2_pre_install_"+datetime.now().strftime('%Y%m%d_%H%M%S'))
    backup.mkdir(parents=True,exist_ok=False)
    for rel in (TARGET_MAIN,TARGET_CMAKE):
        (backup/rel).parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(repo/rel,backup/rel)
    for rel,data in planned.items():
        dest=repo/rel
        dest.parent.mkdir(parents=True,exist_ok=True)
        if isinstance(data,bytes):dest.write_bytes(data)
        else:dest.write_text(data)
    print('M2.4c2 FSM SIM shadow overlay INSTALLED.')
    print(f'Backed up main/CMake BEFORE edits: {backup}')
    print('NO real JOINTCTRL SDK operations added; real JOINTCTRL rejected before DDS Init.')
    print('Existing M2.3 simulation files/legacy M2.4b1 SDK code NOT modified.')

if __name__=='__main__':
    try:main()
    except Exception as exc:
        print(f'[M24c2 FAIL CLOSED] {exc}',file=sys.stderr)
        sys.exit(2)
