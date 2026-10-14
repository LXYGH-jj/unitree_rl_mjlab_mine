#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 "$ROOT/tools/offline_audit.py"
python3 "$ROOT/tests/test_installer_fixture.py"
if [[ $# -gt 0 ]]; then
  REPO=$(realpath "$1")
  if [[ -x "$REPO/deploy/interactive_b2z1/m24c_shadow_integration/run_offline_tests.sh" ]]; then
    bash "$REPO/deploy/interactive_b2z1/m24c_shadow_integration/run_offline_tests.sh" "$REPO"
  fi
  if [[ -x "$REPO/deploy/robots/b2z1/m24b2_jointctrl_offline/run_offline_tests.sh" ]]; then
    bash "$REPO/deploy/robots/b2z1/m24b2_jointctrl_offline/run_offline_tests.sh" "$REPO"
  fi
  if [[ -x "$REPO/deploy/robots/b2z1/m24b3_z1_sdk_passive_probe/run_offline_tests.sh" ]]; then
    bash "$REPO/deploy/robots/b2z1/m24b3_z1_sdk_passive_probe/run_offline_tests.sh"
  fi
fi
printf '%s\n' 'M2.4c2 OFFLINE CHECKS PASS (new FSM C++ must still build in original checkout; NO real robot IO)'
