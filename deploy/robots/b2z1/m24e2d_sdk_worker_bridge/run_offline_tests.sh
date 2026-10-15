#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BD="$(mktemp -d /tmp/m24e2d_build_XXXXXX)"
trap 'rm -rf "$BD"' EXIT
cmake -S "$ROOT" -B "$BD" -DCMAKE_BUILD_TYPE=Debug -DM24E2D_ENABLE_HARDWARE_SDK=OFF
cmake --build "$BD" -j2
M24E2C_MOCK_EXE="$BD/m24e2d_z1_worker" \
  python3 -m unittest discover -v -s "$ROOT/../m24e2c_ipc_planner_integration/tests" -p 'test_integration.py'
M24E2A_MOCK_EXE="$BD/m24e2d_z1_worker" \
  python3 -m unittest discover -v -s "$ROOT/../m24e2a_z1_worker_ipc/tests" -p 'test_ipc.py'
M24E2D_MOCK_EXE="$BD/m24e2d_z1_worker" \
  python3 -m unittest discover -v -s "$ROOT/tests" -p 'test_*.py'
echo 'M2.4e2d MOCK REGRESSION PASS; physical SDK backend NOT compiled/linked/loaded'
