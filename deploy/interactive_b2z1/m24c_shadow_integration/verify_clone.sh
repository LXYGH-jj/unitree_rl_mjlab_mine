#!/usr/bin/env bash
# NO real DDS/SDK/socket calls. Repeatable on the second PC.
set -euo pipefail
ROOT="${1:-$(cd "$(dirname "$0")/../../.." && pwd)}"
HERE="$(cd "$(dirname "$0")" && pwd)"
[[ -f "$ROOT/deploy/robots/b2z1/CMakeLists.txt" ]] || { echo '[ERROR] repository root required' >&2; exit 2; }
for p in \
  deploy/interactive_b2z1/sim_closed_loop/joint_target_guard.cpp \
  deploy/interactive_b2z1/sim_closed_loop/joint_target_guard.h \
  deploy/robots/b2z1/config/config_interactive_arm_ik_m23_portable_sim.yaml \
  deploy/interactive_b2z1/models/b2z1_interactive_policy.onnx; do
  [[ -s "$ROOT/$p" ]] || { echo "[BLOCKER] clone missing/empty: $p" >&2; exit 2; }
  if ! git -C "$ROOT" ls-files --error-unmatch -- "$p" >/dev/null 2>&1; then
      echo "[BLOCKER] file exists but is NOT version-controlled: $p" >&2; exit 2
  fi
done
printf '\n===== exact policy model hash =====\n'
sha256sum "$ROOT/deploy/interactive_b2z1/models/b2z1_interactive_policy.onnx"
printf '\n===== shadow integrated 799D chain =====\n'
bash "$HERE/run_offline_tests.sh" "$ROOT"
printf '\n===== offline vendor JOINTCTRL API/safety =====\n'
bash "$ROOT/deploy/robots/b2z1/m24b2_jointctrl_offline/run_offline_tests.sh" "$ROOT"
printf '\n===== offline Z1 feedback quality =====\n'
bash "$ROOT/deploy/robots/b2z1/m24b3_z1_sdk_passive_probe/run_offline_tests.sh"
echo '[PASS] git-cloned M2.4c offline package; hardware/real controller NOT qualified.'
