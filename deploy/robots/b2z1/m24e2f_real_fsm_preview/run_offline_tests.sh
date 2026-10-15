#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
python3 "$ROOT/tests/test_install_transform.py"
python3 "$ROOT/tests/test_source_guards.py"
REPO="${1:-}"
if [[ -n "$REPO" ]];then
  [[ -f "$REPO/deploy/robots/b2z1/main.cpp" ]] || { echo 'bad repo';exit 2; }
  python3 - "$REPO" <<'PY'
import pathlib,sys
r=pathlib.Path(sys.argv[1]);m=(r/'deploy/robots/b2z1/main.cpp').read_text();c=(r/'deploy/robots/b2z1/CMakeLists.txt').read_text()
assert '#include "State_InteractiveArmRealPreview.h"' in m
assert 'M2.4e2f preview refuses non-loopback network before DDS' in m
assert 'm24e2f_real_fsm_preview/src/State_InteractiveArmRealPreview.cpp' in c
assert 'm24e2e_live_input/src/z1_ipc_client.cpp' in c
assert 'm24e2e_live_input/src/b2z1_live_input.cpp' in c
assert (r/'deploy/robots/b2z1/config/config_m24e2f_real_input_preview_sim.yaml').is_file()
print('E2f production wiring source guards PASS')
PY
fi
echo 'M2.4e2f PREVIEW SOURCE TESTS PASS; native SDK2/ONNX build still required on developer PC'
