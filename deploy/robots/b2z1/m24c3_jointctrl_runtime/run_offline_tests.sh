#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" ]]; then
  echo 'Usage: bash run_offline_tests.sh /absolute/path/to/unitree_rl_mjlab' >&2; exit 2
fi
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SAFETY="$REPO/deploy/robots/b2z1/m24b2_jointctrl_offline"
SDK="$REPO/deploy/thirdparty/z1_sdk"
[[ -f "$SAFETY/src/z1_jointctrl_safety.cpp" ]] || { echo "Missing M2.4b2" >&2; exit 2; }
BUILD="$(mktemp -d /tmp/m24c3_build_XXXXXXXX)"
trap 'rm -rf "$BUILD"' EXIT
c++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread \
  -I"$SAFETY/include" -I"$BASE/include" \
  "$SAFETY/src/z1_jointctrl_safety.cpp" \
  "$BASE/src/z1_jointctrl_session.cpp" \
  "$BASE/tests/test_jointctrl_session.cpp" \
  -o "$BUILD/test_m24c3"
"$BUILD/test_m24c3"
# Compilation checks SDK headers only (no constructor, sockets or threads executed).
c++ -std=c++17 -fsyntax-only -Wno-reorder -I"$SDK/include" -I/usr/include/eigen3 \
  "$BASE/tests/vendor_sdk_api_compile.cpp"
echo 'M2.4c3 official vendor JOINTCTRL API contract: PASS (compile-only, NO robot IO)'
python3 "$BASE/tools/offline_audit.py" "$REPO"
python3 "$BASE/tests/test_installer_fixture.py"
echo 'M2.4c3 OFFLINE TESTS PASS (mock control-sink only, NO vendor send)'
