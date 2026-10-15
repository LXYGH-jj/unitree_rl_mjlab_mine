#!/usr/bin/env bash
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:?Usage: run_offline_tests.sh /absolute/path/to/unitree_rl_mjlab}"
E2D="$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge"
python3 "$SELF/tests/test_patched_sources.py" "$E2D"
BUILD="$(mktemp -d /tmp/m24e2h_test_XXXXXX)"
trap 'rm -rf "$BUILD"' EXIT
c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror     -I"$E2D/include" "$SELF/tests/test_gravity_guard.cpp" -o "$BUILD/ff_test"
"$BUILD/ff_test"
# Existing 13 regression cases, mock only; build still DOES NOT link vendor SDK.
bash "$E2D/run_offline_tests.sh"
echo 'M2.4e2h OFFLINE TESTS PASS; no Z1 SDK loaded or motor I/O'
