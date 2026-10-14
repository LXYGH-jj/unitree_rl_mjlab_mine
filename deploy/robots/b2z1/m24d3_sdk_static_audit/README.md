# M2.4d3 — Official Z1 SDK static receive-path audit (no robot, no SDK runtime)

**Baseline:** `LXYGH-jj/unitree_rl_mjlab_mine`, branch `feature/b2z1-interactive-deploy`, commit `3a7ef8a`.

**Scope:** produce a reproducible, useful disassembly/call-path report for **the developer's actual SDK binary** before designing any SDK integration. NO production controller sources or CMake edited. No real Z1 SDK construction, loading, motor IO, network, firmware, or bypass of `main.cpp` pre-DDS real JOINTCTRL lock. 

## What we know from official supplied headers

- `unitreeArm::sendRecv()` invokes `CtrlComponents::sendRecv()` and is **transmit + receive**; it is NOT passive and may send a motor packet if called on hardware.
- `sendRecvThread` (if started) calls `sendRecv()` at nominal 500 Hz. Mixing it with user `sendRecv()` calls produces more than one owner. No real SDK threads are started by this audit.
- `UDPPort::recv` returns a `size_t`, but public SDK `RecvState` does **not** expose a received packet index or timestamp. Static symbol addresses do not certify source identity, atomic state copies, or freshness.
- ALORE also uses a getter + `sendRecv()` model (but in **LOWCMD**, not JOINTCTRL) and does not supply this missing provenance evidence.

## Prerequisites

Linux x86_64, binutils `nm`, `objdump`, `readelf`, Python 3.8+. `g++` used only for synthetic fixture test. No external Python dependencies.

## Unzip, check & install

```bash
cd ~/下载
unzip m24d3_sdk_static_audit_overlay.zip
cd m24d3_sdk_static_audit_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh
```

## Read-only analyze actual installed SDK

The expected SDK hash below is taken from your M2.4d2 diagnostic output; fail if it silently changes.

```bash
SDK="$REPO/deploy/thirdparty/z1_sdk/lib/libZ1_SDK_x86_64.so"
REPORT=/tmp/m24d3_z1_sdk_report
python3 tools/analyze_sdk_rx.py \
    --sdk "$SDK" \
    --out "$REPORT" \
    --expect-sha256 ecfbdd917016de721983ad2cc0770adb110832f506cc9ef67b8c37a9f5f3eafd
sed -n '1,240p' "$REPORT/REPORT.md"
```

`REPORT.md` contains the direct call/jump targets and assembly snippets around network libc calls. `evidence.json` stores all resolved target function calls; `disassembly/*.asm` contains local function disassembly. No full binary or SDK library is copied to the report.

To focus on the actual receive path (usually most useful to send back):

```bash
grep -n -E 'recvfrom|select|recv\(' "$REPORT/disassembly/udp_recv_"*.asm "$REPORT/disassembly/ctrl_sendrecv.asm" | head -100
sed -n '1,160p' "$REPORT/disassembly/ctrl_sendrecv.asm"
```

## Interpretation guardrails

- It is **not safe** to assert a return from `sendRecv()` means a fresh packet arrived, or to fabricate `RxEvidence` booleans from `recv()` invocation counts / getter times.
- The SDK's receive buffer may be decoded into `lowstate` by internal thread with no synchronization exposed; even a `recvfrom()` event intercepted outside the library may not be paired atomically with the snapshot. This project does **not** hook libc / UDP or modify vendor binary.
- A future qualified solution requires reviewed vendor receive-side source/callback that produces immutable decoded state and receiver-origin peer/time evidence from the *same datagram*, or an independently implemented verified transport for the **actual** protocol (not M2.4d2 mock).
- The historical B2+Z1 fuse issue, mechanical clearances and E-stop remain unresolved. Real Z1 output is locked, irrespective of audit results.

## Deliverable and next engineering decision

Share `REPORT.md` (and if needed `ctrl_sendrecv.asm`, `udp_recv_*.asm`) so we can establish the send/recv/decode call sequence. Then choose whether vendor/source cooperation is required for an atomic receive callback. Do not integrate a real vendor `CommandSink` until independent hardware/electrical and receive-ownership review.

## Git (only after tests)

```bash
cd "$REPO"
git add -- deploy/robots/b2z1/m24d3_sdk_static_audit/
git diff --cached --check
# commit only after confirming report and clean worktree tests; no binaries are added.
```
