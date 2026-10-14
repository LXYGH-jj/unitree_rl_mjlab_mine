#!/usr/bin/env bash
set -euo pipefail
REPO="${1:?Usage: bash install.sh /path/to/unitree_rl_mjlab}"
HERE="$(cd "$(dirname "$0")" && pwd)"
TARGET="$REPO/deploy/interactive_b2z1/m24c_shadow_integration"
[[ -f "$REPO/deploy/robots/b2z1/CMakeLists.txt" && -f "$REPO/deploy/interactive_b2z1/include/interactive_pipeline.h" ]] || {
    echo "[ERROR] Not the expected unitree_rl_mjlab checkout" >&2; exit 2; }
[[ ! -e "$TARGET" ]] || { echo "[ERROR] Already exists; refusing to overwrite: $TARGET" >&2; exit 2; }
mkdir -p "$(dirname "$TARGET")"
cp -a "$HERE" "$TARGET"
echo "Installed OFFLINE shadow adapter: $TARGET"
echo 'No existing CMake/FSM/SDK2/Z1 SDK files modified; NO robot IO path added.'
