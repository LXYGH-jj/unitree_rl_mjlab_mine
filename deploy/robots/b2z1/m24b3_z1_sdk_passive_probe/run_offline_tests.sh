#!/usr/bin/env bash
set -euo pipefail
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP="$(mktemp -d -t m24b3_XXXXXX)"
trap 'rm -rf "$TMP"' EXIT
"${CXX:-g++}" -std=c++17 -Wall -Wextra -Werror -O2 -I"$BASE/include" \
  "$BASE/src/z1_feedback_quality.cpp" "$BASE/tests/test_z1_feedback_quality.cpp" \
  -o "$TMP/test_feedback"
"$TMP/test_feedback"
python3 "$BASE/tools/offline_audit.py" "$BASE"
