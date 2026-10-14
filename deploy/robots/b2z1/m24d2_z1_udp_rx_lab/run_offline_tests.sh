#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" ]]; then echo 'Usage: bash run_offline_tests.sh /absolute/path/to/repo' >&2; exit 2; fi
HERE="$REPO/deploy/robots/b2z1/m24d2_z1_udp_rx_lab"
[[ -f "$HERE/CMakeLists.txt" ]] || { echo "Install M2.4d2 into the REPO before testing" >&2; exit 2; }
[[ -f "$REPO/deploy/robots/b2z1/m24d1_z1_transport/src/z1_rx_validation.cpp" ]] || { echo 'Missing M2.4d1 prereq' >&2; exit 2; }
BUILD="$(mktemp -d /tmp/m24d2_lab_XXXXXXXX)"
trap 'rm -rf "$BUILD"' EXIT
cmake -S "$HERE" -B "$BUILD" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BUILD" -j2
ctest --test-dir "$BUILD" --output-on-failure
python3 "$HERE/tools/offline_audit.py" "$REPO"
python3 "$HERE/tools/sdk_interface_report.py" "$REPO"
python3 "$HERE/tests/test_installer_fixture.py"
echo 'M2.4d2 ALL TESTS PASS: synthetic loopback only; vendor RX proof and physical JOINTCTRL remain LOCKED'
