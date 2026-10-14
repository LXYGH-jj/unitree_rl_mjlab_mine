#!/usr/bin/env bash
set -euo pipefail
REPO="${1:?Usage: bash run_offline_tests.sh /absolute/path/to/unitree_rl_mjlab}"
BASE="$REPO/deploy/robots/b2z1/m24d4_z1_transport_contract"
[[ -f "$BASE/CMakeLists.txt" ]] || { echo 'Not installed in target repository'; exit 2; }
BUILD="$(mktemp -d /tmp/m24d4_test_XXXXXX)"
trap 'rm -rf "$BUILD"' EXIT
cmake -S "$BASE" -B "$BUILD"
cmake --build "$BUILD" -j2
ctest --test-dir "$BUILD" --output-on-failure
python3 "$BASE/tools/offline_audit.py" "$REPO"
python3 "$BASE/tests/test_installer_fixture.py" "$REPO"
echo 'M2.4d4 ALL OFFLINE TESTS PASS; VENDOR SDK RX/JOINTCTRL IO REMAIN LOCKED'
