#!/usr/bin/env bash
set -euo pipefail
REPO="${1:?Usage: bash run_offline_tests.sh /path/to/unitree_rl_mjlab}"
HERE="$(cd "$(dirname "$0")" && pwd)"
CXX="${CXX:-g++}"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
"$CXX" -std=c++17 -Wall -Wextra -Werror -O1 -pthread \
    -I"$HERE/include" -I"$REPO/deploy/interactive_b2z1/include" \
    "$HERE/src/m24c_shadow_bridge.cpp" \
    "$REPO/deploy/interactive_b2z1/src/interactive_observation.cpp" \
    "$REPO/deploy/interactive_b2z1/src/interactive_feedback_adapter.cpp" \
    "$REPO/deploy/interactive_b2z1/src/interactive_snapshot_store.cpp" \
    "$REPO/deploy/interactive_b2z1/src/interactive_pipeline.cpp" \
    "$HERE/tests/test_m24c_shadow.cpp" -o "$TMP/m24c_shadow_test"
"$TMP/m24c_shadow_test"
echo '[PASS] Pure C++ with REAL production 799D observation/adapter/pipeline. No SDK, no DDS, no Z1 control.'
