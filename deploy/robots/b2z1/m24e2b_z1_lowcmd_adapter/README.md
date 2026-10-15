# M2.4e2b — Official Z1 LOWCMD adapter, *source + offline tests only*

**Goal:** pre-build a vendor-SDK-compatible single-owner control bridge for the ALORE-style independent Z1 Worker. **Not yet a working physical Worker.** This overlay does NOT change existing M2.4e2a IPC V1, the B2 controller, simulation, ONNX, FSM, config or real JOINTCTRL hardware guard. No hardware runtime executable is delivered or started.

## What is implemented

- `z1_lowcmd_plan`: offline-testable validated target/feedback + rate/acceleration limiting + watchdog + latched errors. Offline speed limits, acceleration, temperature, TTL and tracking threshold are *development fixtures, NOT approved physical parameters*. No claim of motor stop or power isolation.
- `z1_sdk_bridge`: official Z1 SDK `unitreeArm` source adapter using `setArmCmd`, `sendRecv`, `lowstate` getters and `ArmFSMState::LOWCMD`; refuses `start()`/`exchange()` by default. The prospective enabled implementation is checked under both compile configurations with `-fsyntax-only`, and its runtime remains disabled. When later reviewed for hardware, its SDK call ownership must stay on one thread, vendor background thread stopped before the custom `sendRecv()` loop.
- The returned SDK sample carries only `read_at` (local getter sample timestamp), not verified UDP receive provenance. Do not feed this sample to the old M2.4d1 `RxEvidence` proof gate or forge flags.
- Pure logic tests cover rate+acceleration limits, TTL timeout, replay/out-of-range, motor error and wrong SDK mode.

## Install and run (developer PC)

```bash
cd ~/下载
unzip m24e2b_z1_lowcmd_adapter_overlay.zip
cd m24e2b_z1_lowcmd_adapter_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2b_z1_lowcmd_adapter/run_offline_tests.sh" "$REPO"
```

The script compiles and tests pure C++ logic, and **syntax checks** the SDK adapter against your local installed official headers both with and without the code path macro. No SDK library is loaded/constructed and no B2/Z1 network traffic occurs. It does NOT yet establish vendor ABI/runtime compatibility, and has not been tested against the actual repaired arm.

If your SDK is elsewhere, set `Z1_SDK_ROOT` before running the script. If SDK header syntax check fails, save the compiler error; don't change the SDK itself.

## Engineering work still required before any motor output

1. Merge the SDK adapter into the *independent* Worker, preserving `V1` IPC semantics and a **single owning SDK thread**. Do not start `sendRecvThread` simultaneously with the worker loop. M2.4e2a currently reports `MOCK` and the production worker must report a *distinct*, plainly qualified `SDK_GETTER_UNVERIFIED` status.
2. Add input-side safety: verified model joint bounds, calibrated gains/torque settings, full feedback health validation; the offline numeric limits are not usable as physical values.
3. Implement supervised fixed-bench stop/hold/error procedure that never implicitly calls `backToStart` or claims software stop is an E-stop; recheck electrical repair, support and operator E-stop before running.
4. Separately integrate B2 DDS + Z1 Worker feedback into the 799-D policy. Use actual Z1 measured q/dq, not motor slots 12..17 from the real B2 SDK2 LowState.

Reference code inspected:
- Unitree `z1_sdk/examples/lowcmd_development.cpp`: https://github.com/unitreerobotics/z1_sdk/blob/master/examples/lowcmd_development.cpp
- ALORE `Deployment/Z1_deploy/z1_control.py`: https://github.com/Zhihaibi/ALORE_Legged_Manipulator/blob/main/Deployment/Z1_deploy/z1_control.py

## Narrow git staging

```bash
cd "$REPO"
git add -- deploy/robots/b2z1/m24e2a_z1_worker_ipc/ deploy/robots/b2z1/m24e2b_z1_lowcmd_adapter/
git diff --cached --check
git diff --cached --stat
# commit only after dev-PC tests pass
```

Do not use `git add -A` or stage local backup files.
