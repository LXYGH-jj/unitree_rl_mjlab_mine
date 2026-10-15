#!/usr/bin/env bash
# Compiles and links the physical SDK code; DOES NOT RUN IT. Requires SDK .so on developer machine.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="${1:?Usage: run_sdk_linkcheck.sh /absolute/path/to/repo}"
if [[ ! -f "$REPO/deploy/thirdparty/z1_sdk/include/unitree_arm_sdk/control/unitreeArm.h" ]]; then
  echo 'Missing official Z1 SDK header in repo' >&2; exit 2
fi
BD="$(mktemp -d /tmp/m24e2d_sdk_link_XXXXXX)"
trap 'rm -rf "$BD"' EXIT
cmake -S "$ROOT" -B "$BD" -DCMAKE_BUILD_TYPE=Debug \
  -DM24E2D_ENABLE_HARDWARE_SDK=ON \
  -DSDK_ROOT="$REPO/deploy/thirdparty/z1_sdk"
cmake --build "$BD" -j2
# Do not execute the SDK-enabled binary, even with --help.
echo 'M2.4e2d SDK LINK-CHECK PASS; physical SDK executable NOT launched'
