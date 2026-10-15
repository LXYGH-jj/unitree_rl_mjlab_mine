#!/usr/bin/env bash
# Prepare/transfer deploy source tree to a SEPARATE controller computer. Dry-run by default.
set -euo pipefail
if [[ $# -lt 3 || $# -gt 4 ]]; then
  echo "Usage: transfer_deploy.sh REPO user@host /absolute/remote/repo [--apply]" >&2;exit 2
fi
REPO="$(realpath "$1")"; HOST="$2"; DEST="$3"; MODE="${4:---check}"
[[ -d "$REPO/deploy/robots/b2z1" ]] || { echo 'Not a deploy repo' >&2;exit 2; }
[[ "$HOST" == *@* ]] || { echo 'Target must be a user@host' >&2;exit 2; }
[[ "$DEST" == /* && "$DEST" != '/' ]] || { echo 'Remote repo path must be absolute and non-root' >&2;exit 2; }
[[ "$MODE" == '--check' || "$MODE" == '--apply' ]] || { echo 'Mode must be --check or --apply' >&2;exit 2; }
# No --delete; do not sync .git, compiled binaries or build/cache outputs.
OPTS=(-av --itemize-changes --exclude='build/' --exclude='.git/'   --exclude='__pycache__/' --exclude='*.pyc' --exclude='log/'   --exclude='.venv/' --exclude='*.o' --exclude='*.a')
if [[ "$MODE" != '--apply' ]]; then OPTS+=(--dry-run);fi
printf 'MODE=%s HOST=%s DEST=%s\n' "$MODE" "$HOST" "$DEST"
printf '%s\n' 'Only the repo deploy/ directory is copied. No runtime or motor commands.'
# Require existing remote repo parent directory to avoid shell-escaped mkdir over SSH.
# User must create $DEST beforehand with normal ssh/mkdir permissions.
rsync "${OPTS[@]}" "$REPO/deploy/" "$HOST:$DEST/deploy/"
echo 'Transfer finished/dry-run. Rebuild for TARGET architecture; do not copy dev PC binaries.'
