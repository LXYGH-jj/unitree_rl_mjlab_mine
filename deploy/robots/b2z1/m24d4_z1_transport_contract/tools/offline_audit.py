#!/usr/bin/env python3
"""Static hard-boundary checks; in no way hardware proof."""
import sys
from pathlib import Path

def check(ok, msg):
    if not ok:
        raise RuntimeError("[FAIL] " + msg)
    print("[PASS] " + msg)

def main(repo):
    base = repo / "deploy/robots/b2z1"
    module = base / "m24d4_z1_transport_contract"
    main_cpp = (base / "main.cpp").read_text()
    cmake = (base / "CMakeLists.txt").read_text()
    h = (module / "include/z1_transport_contract.h").read_text()
    cpp = (module / "src/z1_transport_contract.cpp").read_text()
    cm = (module / "CMakeLists.txt").read_text()
    check("REAL Z1 JOINTCTRL hardware path LOCKED" in main_cpp and "return 2;" in main_cpp,
          "Pre-DDS JOINTCTRL real hardware refusal preserved")
    check("m24d4_z1_transport_contract" not in cmake, "Production b2z1_ctrl has no new transport linked")
    check("unitree_arm_sdk" not in h + cpp + cm and "startTrack" not in h + cpp + cm,
          "No vendor header, constructor, mode switch or sendRecv used")
    check("socket(" not in cpp and "sendto(" not in cpp and "recvfrom(" not in cpp,
          "No network socket or physical UDP")
    check("VendorDisabledTransport" in h and "kPhysicalOutputEnabled = false" in h,
          "Vendor output compile-time disabled")
    check("TransportKind::SyntheticMock" in cpp and "runtime refuses all vendor transports" in cpp,
          "Runtime refuses physical SDK implementation")
    check("flock(fd, LOCK_EX | LOCK_NB)" in cpp, "Advisory cross-process owner conflict check exists")
    check("z1_transport_session.h" in h and "gate_.onReceiverCallback" in cpp and "gate_.tick" in cpp,
          "Actual M2.4d1/c3 feedback and output gate reused")
    check("m24d1_z1_transport/src/z1_transport_session.cpp" in cm,
          "Standalone mock test compiles real M2.4d1/c3 sources")
    print("M2.4d4 OFFLINE STATIC BOUNDARY: PASS; no real SDK or robot IO")

if __name__ == "__main__":
    if len(sys.argv) != 2: raise SystemExit("usage: offline_audit.py /absolute/repo")
    main(Path(sys.argv[1]).resolve())
