#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" ]];then echo 'Usage: bash run_offline_tests.sh /absolute/path/to/repo' >&2;exit 2;fi
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK="$REPO/deploy/thirdparty/z1_sdk"
SAFETY="$REPO/deploy/robots/b2z1/m24b2_jointctrl_offline"
SESSION="$REPO/deploy/robots/b2z1/m24c3_jointctrl_runtime"
[[ -f "$SAFETY/src/z1_jointctrl_safety.cpp" && -f "$SESSION/src/z1_jointctrl_session.cpp" ]] || { echo 'Missing M2.4b2/c3 dependencies' >&2;exit 2;}
BUILD="$(mktemp -d /tmp/m24d1_offline_XXXXXXXX)"
trap 'rm -rf "$BUILD"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
    -I"$SAFETY/include" -I"$SESSION/include" -I"$BASE/include" \
    "$SAFETY/src/z1_jointctrl_safety.cpp" \
    "$SESSION/src/z1_jointctrl_session.cpp" \
    "$BASE/src/z1_rx_validation.cpp" \
    "$BASE/src/z1_transport_session.cpp" \
    "$BASE/tests/test_transport_session.cpp" -o "$BUILD/m24d1_test"
"$BUILD/m24d1_test"
c++ -std=c++17 -fsyntax-only -Wno-reorder \
    -I"$SDK/include" -I/usr/include/eigen3 \
    -I"$SAFETY/include" -I"$SESSION/include" -I"$BASE/include" \
    "$BASE/tests/vendor_sdk_abi_compile.cpp" "$BASE/src/z1_sdk_getter_snapshot.cpp"
echo 'M2.4d1 vendor SDK ABI and getter adapter: PASS (compile-only, NO socket)'
python3 "$BASE/tests/test_installer_fixture.py"
python3 "$BASE/tools/offline_audit.py" "$REPO"
if [[ -f "$SDK/lib/libZ1_SDK_x86_64.so" ]]; then
    python3 "$BASE/tools/sdk_abi_audit.py" "$SDK/lib/libZ1_SDK_x86_64.so"
else
    echo '[INFO] Vendor binary unavailable for ELF report; real build required in developer repo'
fi
echo 'M2.4d1 OFFLINE TESTS PASS; REAL SDK RX/JOINTCTRL OUTPUT REMAIN LOCKED'
