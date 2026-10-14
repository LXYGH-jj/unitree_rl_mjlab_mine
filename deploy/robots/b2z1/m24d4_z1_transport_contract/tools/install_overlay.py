#!/usr/bin/env python3
"""M2.4d4 conservative installer: create one isolated directory only."""
import argparse
import shutil
from pathlib import Path

SUBDIR = Path("deploy/robots/b2z1")
NEEDED = (
    "m24b2_jointctrl_offline/include/z1_jointctrl_safety.h",
    "m24c3_jointctrl_runtime/include/z1_jointctrl_session.h",
    "m24d1_z1_transport/include/z1_transport_session.h",
    "m24d2_z1_udp_rx_lab/CMakeLists.txt",
    "m24d3_sdk_static_audit/tools/analyze_sdk_rx.py",
    "main.cpp",
)

def install(source: Path, repo: Path, apply: bool):
    if not repo.is_dir() or not repo.is_absolute():
        raise ValueError("Target repository directory must be an absolute existing path")
    parent = repo / SUBDIR
    absent = [x for x in NEEDED if not (parent / x).is_file()]
    if absent:
        raise RuntimeError("M2.4d4 prerequisites missing: " + ", ".join(absent))
    main = (parent / "main.cpp").read_text()
    if "REAL Z1 JOINTCTRL hardware path LOCKED" not in main or "return 2;" not in main:
        raise RuntimeError("Pre-DDS hardware guard not found; refusing installation")
    dest = parent / "m24d4_z1_transport_contract"
    if dest.exists():
        raise FileExistsError(f"Already installed; refusal to overwrite: {dest}")
    print("M2.4d4 will ADD ONLY:", dest)
    print("UNCHANGED: CMake/main/FSM, B2 SDK2, official Z1 SDK, M2.3, M2.4c2/c3/d1/d2/d3")
    print("VENDOR OUTPUT: hard-disabled, synthetic mock tests only")
    if apply:
        shutil.copytree(source, dest, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
        print("M2.4d4 INSTALL SUCCESS:", dest)
    else:
        print("DRY RUN: no files changed")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("repo", type=Path)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    install(Path(__file__).resolve().parents[1], args.repo.resolve(), args.apply)
