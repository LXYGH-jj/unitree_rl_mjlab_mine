#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" || ! -d "$REPO/deploy/robots/b2z1" ]]; then
  echo "Usage: bash install.sh /path/to/unitree_rl_mjlab" >&2
  exit 2
fi
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$REPO/deploy/robots/b2z1/m24b2_jointctrl_offline"
if [[ -e "$DEST" ]]; then
  echo "REFUSE overwrite: $DEST" >&2
  exit 3
fi
# IMPORTANT: Do not overwrite the M2.4b1 LOWCMD bridge or existing CMake/FSM.
cp -a "$BASE" "$DEST"
echo "Installed independent M2.4b2 JOINTCTRL offline module: $DEST"
echo "No tracked source / CMake / B2 communication / config files were modified."
echo "Next: bash '$DEST/run_offline_tests.sh' '$REPO'"
