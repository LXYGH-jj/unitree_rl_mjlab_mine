#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BD="$(mktemp -d /tmp/m24e2b_planner_build_XXXXXX)"
trap 'rm -rf "$BD"' EXIT
cmake -S "$ROOT" -B "$BD" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BD" -j2
ctest --test-dir "$BD" --output-on-failure
# Production SDK headers are required only for syntax check. This does not load SDK.
SDK_ROOT="${Z1_SDK_ROOT:-}"
if [[ -z "$SDK_ROOT" && -n "${1:-}" ]]; then
  candidate="$1/deploy/thirdparty/z1_sdk"
  [[ -f "$candidate/include/unitree_arm_sdk/control/unitreeArm.h" ]] && SDK_ROOT="$candidate" || true
fi
if [[ -n "$SDK_ROOT" ]]; then
  [[ -f "$SDK_ROOT/include/unitree_arm_sdk/control/unitreeArm.h" ]] || { echo "Z1_SDK_ROOT invalid: $SDK_ROOT" >&2; exit 2; }
  echo "SDK header syntax-check only (physical path macro OFF and ON, no linking or SDK load)"
  for opt in '' '-DM24E2B_ENABLE_HARDWARE_SDK'; do
    # The vendor SDK contains a known member-initialization-order warning.
    # Treat only its public headers as SYSTEM; local adapter stays strict.
    c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -fsyntax-only $opt \
       -I"$ROOT/include" -isystem "$SDK_ROOT/include" -I/usr/include/eigen3 \
       "$ROOT/src/z1_sdk_bridge.cpp"
  done
  echo "SDK header checks PASS"
else
  echo "SDK header syntax-check SKIPPED: set Z1_SDK_ROOT or pass REPO path; pure C++ tests still valid."
fi
printf '%s\n' 'M2.4e2b OFFLINE TESTS PASS; no SDK runtime, DDS, UDP or robot control executed.'
