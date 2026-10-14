#!/usr/bin/env python3
import tempfile
import subprocess
import sys
from pathlib import Path

def run(args, expected=0):
    p=subprocess.run(args,text=True,capture_output=True)
    if (p.returncode==0)!=(expected==0):
        raise RuntimeError(f"Unexpected installer status {p.returncode}: {p.stdout}\n{p.stderr}")

if __name__=="__main__":
    repo=Path(sys.argv[1]);source=Path(__file__).resolve().parents[1]
    installer=source/"tools/install_overlay.py"
    with tempfile.TemporaryDirectory(prefix="m24d4_installer_") as temp:
        fake=Path(temp)/"repo"; parent=fake/"deploy/robots/b2z1"
        for relative in (
            "m24b2_jointctrl_offline/include/z1_jointctrl_safety.h",
            "m24c3_jointctrl_runtime/include/z1_jointctrl_session.h",
            "m24d1_z1_transport/include/z1_transport_session.h",
            "m24d2_z1_udp_rx_lab/CMakeLists.txt",
            "m24d3_sdk_static_audit/tools/analyze_sdk_rx.py",
        ):
            (parent/relative).parent.mkdir(parents=True,exist_ok=True)
            (parent/relative).write_text("test")
        (parent/"main.cpp").write_text('"REAL Z1 JOINTCTRL hardware path LOCKED"; return 2;')
        (parent/"CMakeLists.txt").write_text("untouched")
        run([sys.executable,str(installer),str(fake),"--check"])
        assert not (parent/"m24d4_z1_transport_contract").exists()
        run([sys.executable,str(installer),str(fake),"--apply"])
        assert (parent/"m24d4_z1_transport_contract/tests/test_mock_contract.cpp").is_file()
        assert (parent/"CMakeLists.txt").read_text()=="untouched"
        run([sys.executable,str(installer),str(fake),"--apply"],expected=1)
        run([sys.executable,str(installer),str(fake),"--check"],expected=1)
    print("M2.4d4 isolated installer dry-run, apply and repeat guard: PASS")
