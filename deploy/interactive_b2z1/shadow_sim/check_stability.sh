#!/usr/bin/env bash
# Read-only simulation regression; does not start b2z1_ctrl or command actuators.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$ROOT"
RUNS="${1:-6}"
FRAMES="${2:-250}"
BUILD_DIR="${3:-deploy/interactive_b2z1/build/shadow_sim_localdds}"
EXE="$BUILD_DIR/b2z1_interactive_shadow_sim"
MODEL="deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx"
ORT_LIB="$ROOT/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib"
LOG_DIR="deploy/interactive_b2z1/build/shadow_localdds_regression"
if [[ ! "$RUNS" =~ ^[1-9][0-9]*$ || ! "$FRAMES" =~ ^[1-9][0-9]*$ ]]; then
  echo 'Usage: bash check_stability.sh [runs=6] [frames=250] [build_dir]' >&2; exit 2
fi
[[ -x "$EXE" && -f "$MODEL" ]] || { echo "Missing executable ($EXE) or ONNX ($MODEL)" >&2; exit 2; }
mkdir -p "$LOG_DIR"
export LD_LIBRARY_PATH="/usr/local/lib:$ORT_LIB:${LD_LIBRARY_PATH:-}"
echo '=== Loaded CycloneDDS libraries ==='
ldd "$EXE" | grep -E 'libddsc(xx)?\.so' | tee "$LOG_DIR/dds_libraries.txt"
python3 - "$LOG_DIR/dds_libraries.txt" <<'PY'
import pathlib,sys
s=pathlib.Path(sys.argv[1]).read_text()
for lib in ('libddsc.so.0','libddscxx.so.0'):
    matches=[line for line in s.splitlines() if line.strip().startswith(lib+' ')]
    if len(matches)!=1 or '/usr/local/lib/' not in matches[0]:
        raise SystemExit(f'BAD DDS runtime resolution: {lib} -> {matches}')
print('DDS runtime libraries: SAME PREFIX (/usr/local/lib)')
PY
for (( i=1; i<=RUNS; i++ )); do
  csv="$LOG_DIR/run_${i}.csv"
  err="$LOG_DIR/run_${i}.stderr.log"
  echo "=== Run $i/$RUNS (${FRAMES} frames) ==="
  if "$EXE" "$MODEL" "$FRAMES" >"$csv" 2>"$err"; then
    python3 - "$csv" "$FRAMES" "$i" <<'PY'
import csv, math, statistics, sys
path, count, run = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
with open(path, newline='') as fp:
    reader = csv.DictReader(fp)
    expected={'frame','b2_q0','sim_z1_q0','leg_action0','max_abs_leg_action','step_ms'}
    if set(reader.fieldnames or ()) != expected:
        raise SystemExit(f'Unexpected columns: {reader.fieldnames}')
    rows=list(reader)
if len(rows)!=count:
    raise SystemExit(f'Run {run}: expected {count} frames, got {len(rows)}')
for i,r in enumerate(rows):
    if int(r['frame'])!=i:
        raise SystemExit(f'Run {run}: wrong frame index at row {i}')
    if not all(math.isfinite(float(v)) for k,v in r.items() if k!='frame'):
        raise SystemExit(f'Run {run}: nonfinite data at frame {i}')
ms=sorted(float(r['step_ms']) for r in rows)
p95=ms[min(len(ms)-1,math.ceil(.95*len(ms))-1)]
print(f'run={run} frames={len(rows)} mean_ms={statistics.mean(ms):.3f} p95_ms={p95:.3f} max_ms={max(ms):.3f}')
PY
  else
    status=$?
    echo "FAILED run=$i exit=$status ; stderr=$err"
    tail -n 60 "$err" || true
    exit "$status"
  fi
done
echo "M1.4c NORMAL SHADOW REGRESSION: PASS (${RUNS} runs x ${FRAMES} frames)"
echo "Logs: $ROOT/$LOG_DIR"
