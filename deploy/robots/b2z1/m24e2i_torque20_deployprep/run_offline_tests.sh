#!/usr/bin/env bash
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:?Usage: run_offline_tests.sh /absolute/path/to/repo}"
HEADER="$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/include"
B="$(mktemp -d /tmp/m24e2i_torque_XXXXXX)"
trap 'rm -rf "$B"' EXIT
c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -O2 -I"$HEADER" \
  "$SELF/tests/test_torque20.cpp" -o "$B/test_torque20"
"$B/test_torque20"
python3 "$SELF/tools/preflight_target.py" "$REPO" --software-only
