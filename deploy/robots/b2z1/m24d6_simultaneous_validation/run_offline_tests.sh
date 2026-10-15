#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 "$ROOT/tests/test_analyze.py"
python3 -m py_compile "$ROOT/tools/analyze_sim_overlap.py" "$ROOT/tools/install_overlay.py"
echo 'M2.4d6 read-only analyzer offline tests: PASS (no controller changes)'
