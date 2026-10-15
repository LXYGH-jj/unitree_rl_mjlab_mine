#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD="$(mktemp -d -t m24e2a_build_XXXXXXXX)"
trap 'rm -rf -- "$BUILD"' EXIT
cmake -S "$HERE" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release
cmake --build "$BUILD" --parallel 2
M24E2A_MOCK_EXE="$BUILD/m24e2a_z1_mock_worker" python3 -m unittest discover -s "$HERE/tests" -v
printf '\nM2.4e2a IPC/mock tests PASS; vendor SDK never loaded and hardware output not implemented.\n'
