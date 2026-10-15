#!/usr/bin/env python3
"""Add-only M2.4e2c overlay installer; no production source changes."""
import argparse
import shutil
import subprocess
from pathlib import Path


def git(repo, *args):
    return subprocess.check_output(["git", "-C", str(repo), *args], text=True).strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("repo", type=Path)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--apply", action="store_true")
    args = ap.parse_args()
    repo = args.repo.resolve(strict=True)
    root = repo / "deploy/robots/b2z1"
    dest = root / "m24e2c_ipc_planner_integration"
    src = Path(__file__).resolve().parents[1]
    if not (root / "main.cpp").is_file():
        raise RuntimeError("B2Z1 deploy root missing")
    for rel in (
        "m24e2a_z1_worker_ipc/src/z1_worker_mock.cpp",
        "m24e2a_z1_worker_ipc/tests/test_ipc.py",
        "m24e2b_z1_lowcmd_adapter/include/z1_lowcmd_plan.h",
        "m24e2b_z1_lowcmd_adapter/src/z1_lowcmd_plan.cpp",
        "m24e2b_z1_lowcmd_adapter/src/z1_sdk_bridge.cpp",
    ):
        if not (root / rel).is_file():
            raise RuntimeError(f"M2.4e2a/e2b prerequisite not installed: {rel}")
    branch = git(repo, "branch", "--show-current")
    if branch not in ("feature/b2z1-interactive-deploy", "feature/b2z1-alore-real-deploy"):
        raise RuntimeError(f"wrong branch {branch}")
    subprocess.run(["git", "-C", str(repo), "merge-base", "--is-ancestor", "f2a3567", "HEAD"], check=True)
    if dest.exists() or dest.is_symlink():
        raise RuntimeError(f"destination exists (refuse overwrite): {dest}")
    for f in ("src/z1_worker_integrated.cpp", "CMakeLists.txt", "run_offline_tests.sh"):
        if not (src / f).is_file():
            raise RuntimeError(f"overlay payload missing: {f}")
    print(f"BASE f2a3567 ancestor verified; branch={branch}")
    print(f"M2.4e2a + M2.4e2b dependencies found; ADD ONLY {dest}")
    print("No B2/FSM/SDK/ONNX/CMake parent modifications; hardware I/O not compiled or loaded")
    if args.check:
        print("DRY RUN: no changes")
    else:
        shutil.copytree(src, dest, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        print("M2.4e2c INSTALLED:", dest)


if __name__ == "__main__":
    main()
