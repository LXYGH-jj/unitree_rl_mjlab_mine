# M2.4d1: Z1 SDK Feedback Provenance Contract & Transport Session (offline-only)

**Target branch**: `feature/b2z1-interactive-deploy`, based on verified commit `aa0dcc6`.

This module genuinely routes receiver-validated mock Z1 feedback to the existing M2.4c3 `JointCtrlSession` and emits its bounded `(q,qd)` commands into a **mock-only** sink. It also compiles an *unverified* vendor SDK getter snapshot adapter and inspects SDK ELF symbols. It does NOT implement a physical motor sink, SDK startup, `startTrack`, `sendRecv` calls, or a direct receive callback. Thus **it is not real Z1 SDK control**. All hardware modes remain refused before B2 DDS initialization.

## Architecture

`external independently audited RX callback (NOT IMPLEMENTED)` -> `RxEvidence` & snapshot -> `RxValidator` -> `SafetyCore` + `JointCtrlSession` -> *injected mock CommandSink*.

The official prebuilt `libZ1_SDK` publicly exposes getter state and `UDPPort::recv` declarations, **but provides no verified public packet callback/sequence interface in the supplied headers**. `copySdkGetterSnapshotUnverified()` is deliberately tagged `Unverified` and cannot issue a valid RX event. `sampled_at` is NOT a receive timestamp. It is also NOT safe to sample vendor `lowstate` from a concurrent SDK thread without proven synchronization.

The `RxEvidence` C++ structure is a contract for an audited SDK receive-path hook, **not proof by itself**: a caller could invent booleans. Do NOT set callback/peer/matching fields from `sendRecv` loop iterations, SDK getters or local clocks. No production caller supplying this contract exists in this package. The mock tests supply synthetic packet events and are NOT hardware evidence.

### Checks added

- Require explicit verified receive callback provenance, expected UDP peer and proof snapshot came from *same* datagram.
- Strictly increasing receive index + receive completion timestamp; max receive age and decode skew.
- q, dq, tau finite; reviewed q limits; motor error/connection flags **zero**, and explicit per-joint temperature limits.
- Require reported JOINTCTRL and connected SDK for a verified frame.
- Any RX validation fault latches, disarms M2.4c3; a getter-only poll permanently latches this session as ineligible (preventing reauthorization with previously accepted RX samples), and never fabricates freshness.
- One `TransportSession` mutex serializes feedback, authorization, IK goals and output gate. This does NOT enforce interprocess exclusive ownership of `z1_ctrl` / UDP endpoint; implement and validate that separately before hardware IO.

## Developer computer: install

```
cd ~/下载
unzip m24d1_z1_transport_overlay.zip
cd m24d1_z1_transport_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh "$REPO"
cmake -S "$REPO/deploy/robots/b2z1" -B "$REPO/deploy/robots/b2z1/build"
cmake --build "$REPO/deploy/robots/b2z1/build" -j2 --target b2z1_ctrl
```

An original-CMake backup is placed under `~/lxy/b2z1_backups/`, outside Git checkout. Installer refuses duplicate installation, missing prerequisites, or absent exact CMake anchors. It modifies only `deploy/robots/b2z1/CMakeLists.txt` and adds `deploy/robots/b2z1/m24d1_z1_transport`. Does not edit main, FSM, Z1 SDK binary, B2 SDK2, M2.3, M2.4c2 or M2.4c3.

## Important blocker: genuine UDP freshness

On the actual dev PC, `tools/sdk_abi_audit.py` prints ELF dynamic symbols for the supplied Z1 SDK. Merely observing `recvfrom` or `UDPPort::recv` in ELF **does not establish a new-datagram proof**. The next substage requires audited receiver-side instrumentation or verified vendor SDK source/patch providing atomic decoded packet snapshots, origin peer and receive timestamps, and documented lock ownership with its `sendRecvThread`. Before that **do not** make `confirmed_new_packet=true` with synthetic or inferred values in real runs.

The historical fuse failure additionally requires electrical inspection, safe mechanical support, motor configuration and E-stop review. Software `disarm` is not a physical controlled stop.

## Git staging (after developer compile + fresh-worktree verification)

```
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
git add -- deploy/robots/b2z1/CMakeLists.txt deploy/robots/b2z1/m24d1_z1_transport/
git diff --cached --check
git --no-pager diff --cached --stat
```
