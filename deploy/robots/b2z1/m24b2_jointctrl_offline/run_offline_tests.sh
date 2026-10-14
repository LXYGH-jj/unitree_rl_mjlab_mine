#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" || ! -d "$REPO/deploy/robots/b2z1" ]]; then
  echo "Usage: bash run_offline_tests.sh /path/to/unitree_rl_mjlab" >&2
  exit 2
fi
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INCLUDE="$REPO/deploy/thirdparty/z1_sdk/include"
if [[ ! -f "$INCLUDE/unitree_arm_sdk/control/unitreeArm.h" ]]; then
  echo "Vendor Z1 SDK headers not found: $INCLUDE" >&2
  exit 2
fi
TMP="$(mktemp -d -t m24b2_jointctrl_XXXXXX)"
trap 'rm -rf "$TMP"' EXIT
CXX="${CXX:-g++}"
"$CXX" -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 \
  -I"$BASE/include" \
  "$BASE/src/z1_jointctrl_safety.cpp" "$BASE/tests/test_jointctrl_safety.cpp" \
  -o "$TMP/test_jointctrl_safety"
"$TMP/test_jointctrl_safety"
# Validate real SDK JOINTCTRL C++ signatures but NEVER load the shared library,
# create a unitreeArm instance, start z1_ctrl, open a socket or transmit a command.
"$CXX" -std=c++17 -fsyntax-only \
  -I"$BASE/include" -I"$INCLUDE" -I/usr/include/eigen3 \
  "$BASE/src/z1_jointctrl_sdk_candidate.cpp" "$BASE/tests/test_vendor_no_io.cpp"
echo "M2.4b2 vendor SDK API: PASS (compile-only, no linking/running)"
echo "[GUARD] Candidate SDK functions are all locked; B2 SDK2/FSM unaffected"
