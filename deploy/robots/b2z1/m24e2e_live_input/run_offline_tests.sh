#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
REPO="${1:-}"
BD="$(mktemp -d /tmp/m24e2e_test_XXXXXX)"
trap 'rm -rf "$BD"' EXIT
args=()
if [[ -n "$REPO" ]]; then
  [[ -f "$REPO/deploy/interactive_b2z1/src/interactive_observation.cpp" ]] || { echo "Bad REPO path" >&2; exit 2; }
  args+=("-DREPO_ROOT=$REPO")
fi
cmake -S "$ROOT" -B "$BD/build" -DCMAKE_BUILD_TYPE=Debug "${args[@]}"
cmake --build "$BD/build" -j2
"$BD/build/m24e2e_live_test"
# Use the current integrated M2.4e2d MOCK worker as the IPC test server.
if [[ -n "$REPO" ]]; then
  WORKER_DIR="$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge"
else
  WORKER_DIR="/mnt/data/m24e2d_sdk_worker_bridge_overlay"
fi
if [[ ! -f "$WORKER_DIR/CMakeLists.txt" ]]; then
  echo "M2.4e2d worker not found" >&2; exit 2
fi
cmake -S "$WORKER_DIR" -B "$BD/wbuild" -DM24E2D_ENABLE_HARDWARE_SDK=OFF
cmake --build "$BD/wbuild" -j2
SOCKET="$BD/z1.sock"
"$BD/wbuild/m24e2d_z1_worker" --socket "$SOCKET" --backend mock >"$BD/worker.log" 2>&1 &
wpid=$!
trap 'kill "$wpid" 2>/dev/null || true; wait "$wpid" 2>/dev/null || true; rm -rf "$BD"' EXIT
for _ in {1..100}; do [[ -S "$SOCKET" ]] && break; sleep .01; done
[[ -S "$SOCKET" ]] || { cat "$BD/worker.log"; exit 2; }
"$BD/build/m24e2e_client_test" "$SOCKET"
"$BD/build/m24e2e_mock_j5_control_test" "$SOCKET"
python3 "$ROOT/tests/test_source_guards.py" "$REPO"
echo 'M2.4e2e ALL OFFLINE TESTS PASS; no DDS, vendor SDK or motor IO'
