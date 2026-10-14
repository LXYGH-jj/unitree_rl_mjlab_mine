#!/usr/bin/env bash
set -euo pipefail
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -m py_compile "$BASE/tools/analyze_sdk_rx.py" "$BASE/tools/install_overlay.py"
python3 "$BASE/tests/test_static_audit.py"
python3 - "$BASE" <<'PY'
from pathlib import Path
import sys
b = Path(sys.argv[1])
s = (b/'tools/analyze_sdk_rx.py').read_text()
assert 'ctypes.CDLL' not in s and 'import socket' not in s
assert 'subprocess.run(args, check=True' in s
assert '"-d"' in s and '"-C"' in s
assert 'certifies_new_packet_proof": False' in s
x = (b/'tools/install_overlay.py').read_text()
assert 'shutil.copytree' in x
assert 'CMakeLists.txt' not in x
print('M2.4d3 static script guard: PASS (no SDK execution, no project CMake changes)')
PY
