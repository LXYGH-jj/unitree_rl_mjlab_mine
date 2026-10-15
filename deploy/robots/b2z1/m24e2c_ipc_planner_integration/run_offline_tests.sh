#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BD="$(mktemp -d /tmp/m24e2c_mock_build_XXXXXX)"
trap 'rm -rf "$BD"' EXIT
cmake -S "$ROOT" -B "$BD" -DCMAKE_BUILD_TYPE=Debug
cmake --build "$BD" -j2
M24E2C_MOCK_EXE="$BD/m24e2c_z1_integrated_mock_worker" \
  python3 -m unittest discover -v -s "$ROOT/tests" -p 'test_*.py'
# Regression against the exact already validated IPC V1 test suite.
M24E2A_MOCK_EXE="$BD/m24e2c_z1_integrated_mock_worker" \
  python3 -m unittest discover -v -s "$ROOT/../m24e2a_z1_worker_ipc/tests" -p 'test_ipc.py'
echo 'M2.4e2c IPC+Planner MOCK INTEGRATION PASS; NO REAL SDK AND NO ROBOT IO'
