#!/usr/bin/env bash
set -euo pipefail
REPO="${1:-}"
if [[ -z "$REPO" || ! -d "$REPO/deploy/robots/b2z1/m24b2_jointctrl_offline" ]]; then
 echo "Usage: bash install.sh /path/to/unitree_rl_mjlab (M2.4b2 must be installed)" >&2
 exit 2
fi
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST="$REPO/deploy/robots/b2z1/m24b3_z1_sdk_passive_probe"
if [[ -e "$DEST" ]]; then echo "REFUSE overwrite: $DEST" >&2; exit 3; fi
cp -a "$BASE" "$DEST"
rm -rf "$DEST/build"
echo "Installed standalone M2.4b3 module: $DEST"
echo "No tracked project files, B2 SDK2 code, FSM, or legacy LOWCMD files modified."
