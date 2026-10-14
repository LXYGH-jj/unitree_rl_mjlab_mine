#!/usr/bin/env bash
set -euo pipefail
REPO="${1:?usage: bash run_offline_tests.sh /abs/path/to/unitree_rl_mjlab}"
MODULE="$REPO/deploy/robots/b2z1/m24d5_jointctrl_mock_fsm"
[[ -d "$MODULE" ]] || { echo "M2.4d5 not installed" >&2; exit 2; }
B="$(mktemp -d /tmp/m24d5_lab_XXXXXXXX)"
trap 'rm -rf "$B"' EXIT
cmake -S "$MODULE" -B "$B"
cmake --build "$B" -j2
ctest --test-dir "$B" --output-on-failure
python3 "$MODULE/tools/offline_audit.py" "$REPO"
python3 "$MODULE/tests/test_installer_fixture.py" "$MODULE"
echo 'M2.4d5 OFFLINE TESTS PASS: mock gate and static FSM integration. Real b2z1_ctrl compile and MuJoCo joint motion must be verified on workstation.'
