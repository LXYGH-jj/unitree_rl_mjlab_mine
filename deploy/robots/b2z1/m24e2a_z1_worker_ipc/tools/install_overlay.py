#!/usr/bin/env python3
"""Only adds an isolated mock-only M2.4e2a folder. No production changes."""
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
    dest = root / "m24e2a_z1_worker_ipc"
    src = Path(__file__).resolve().parents[1]
    assert (root / "main.cpp").is_file(), "B2Z1 deploy root missing"
    branch = git(repo, "branch", "--show-current")
    assert branch in ("feature/b2z1-interactive-deploy", "feature/b2z1-alore-real-deploy"), (
        f"wrong branch {branch}; use the B2Z1 branch")
    assert subprocess.run(["git", "-C", str(repo), "merge-base", "--is-ancestor", "f2a3567", "HEAD"]).returncode == 0, "f2a3567 baseline missing"
    assert not dest.exists() and not dest.is_symlink(), f"destination already exists: {dest}"
    assert (src / "src/z1_worker_mock.cpp").is_file()
    print(f"BASE f2a3567 checked; branch={branch}")
    print(f"ADD ONLY {dest}")
    print("No production B2/FSM/CMake/YAML/ONNX or SDK change. No physical IO.")
    if args.check:
        print("DRY RUN: unchanged")
    else:
        shutil.copytree(src, dest, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        print("INSTALLED", dest)


if __name__ == "__main__":
    main()
