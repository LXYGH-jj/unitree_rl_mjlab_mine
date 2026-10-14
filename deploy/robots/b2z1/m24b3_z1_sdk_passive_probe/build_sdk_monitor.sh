#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" || ! -f "$REPO/deploy/thirdparty/z1_sdk/include/unitree_arm_sdk/control/unitreeArm.h" ]]; then
  echo "Usage: bash build_sdk_monitor.sh /path/to/unitree_rl_mjlab" >&2
  exit 2
fi
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK="$REPO/deploy/thirdparty/z1_sdk"
LIB="$SDK/lib/libZ1_SDK_x86_64.so"
if [[ ! -f "$LIB" ]]; then echo "Missing vendor SDK: $LIB" >&2; exit 2; fi
OUT="$BASE/build"
mkdir -p "$OUT"
"${CXX:-g++}" -std=c++17 -O2 -Wall -Wextra -pthread \
  -I"$BASE/include" -I"$SDK/include" -I/usr/include/eigen3 \
  "$BASE/src/z1_feedback_quality.cpp" "$BASE/src/z1_sdk_passive_probe.cpp" \
  "$LIB" -Wl,-rpath,"$SDK/lib" -o "$OUT/z1_sdk_passive_probe"
# This --help path does not construct vendor SDK or access any device.
"$OUT/z1_sdk_passive_probe" --help
printf '%s\n' "Build succeeded; real Z1 test NOT run."
