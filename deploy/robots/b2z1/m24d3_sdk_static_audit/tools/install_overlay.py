#!/usr/bin/env python3
"""Install only static audit scripts under a new directory; never edit CMake or hardware code."""
import argparse
import shutil
from pathlib import Path

DIR = Path(__file__).resolve().parents[1]
REQUIRED = [
    "deploy/robots/b2z1/main.cpp",
    "deploy/robots/b2z1/m24d1_z1_transport/include/z1_rx_validation.h",
    "deploy/robots/b2z1/m24d2_z1_udp_rx_lab/README.md",
    "deploy/thirdparty/z1_sdk/lib/libZ1_SDK_x86_64.so",
]
DEST = Path("deploy/robots/b2z1/m24d3_sdk_static_audit")


def install(repo, apply):
    repo = repo.resolve()
    if not repo.is_dir() or any(not (repo / p).is_file() for p in REQUIRED):
        raise RuntimeError("M2.4d3 prerequisites missing; need repo with M2.4d1+d2 and local SDK")
    if (repo / DEST).exists():
        raise RuntimeError("M2.4d3 module already installed; no overwrite")
    print("M2.4d3 new static-audit-only directory:", repo / DEST)
    print("NO CHANGES to CMake/main/FSM/vendor SDK/B2 binaries; NO SDK runtime use")
    if apply:
        shutil.copytree(DIR, repo / DEST, ignore=shutil.ignore_patterns('__pycache__', '*.pyc'))
        print("M2.4d3 STATIC AUDIT TOOLS INSTALLED")
    else:
        print("DRY RUN: no files changed")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("repo", type=Path)
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--apply", action="store_true")
    a = p.parse_args()
    install(a.repo, a.apply)
