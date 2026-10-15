#!/usr/bin/env bash
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHONDONTWRITEBYTECODE=1 python3 "$DIR/tests/test_install_logic.py"
python3 -m py_compile "$DIR/tools/install_overlay.py"
echo 'M2.4e2g OFFLINE SOURCE/CONFIG TESTS PASS; native compile required on developer PC; NO robot IO'
