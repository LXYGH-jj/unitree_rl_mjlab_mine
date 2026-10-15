# M2.4e2f Interactive real-input FSM PREVIEW (SIM ONLY)

**Purpose:** integrate E2e IPC + 799D observation + actual ONNX inference + Z1 SDK FK/IK
into the native B2 FSM plumbing WITHOUT commanding the real B2 or Z1. This is the
next **compiled integration step**, not a hardware-control completion claim.

## Strict limits
- Only with `--network lo` and the separate `--backend mock` Z1 Worker.
- New FSM generates **no** motor target; `post_run()` is deliberately empty.
- The base B2 FSM/stand controller may have commanded motors **before entering**
  the preview FSM. NEVER use the preview config on physical B2.
- B2 timestamp here is first-observed monotonic DDS `tick()` time in the FSM,
  **NOT** a callback arrival timestamp. The next real hardware step must modify
  or wrap the actual B2 DDS subscriber callback to record reception time.
- Z1 `ipc_received_at` is an IPC reply time, **NOT** proven SDK packet reception.
- Z1 worker `Session::target()` is SDK-disabled; this stage does NOT solve the
  physical LOWCMD E-stop / hold/passive behavior.
- Z1 mock dynamics have no actual motor lag; don't claim sim-to-real validation.

## Installation

```
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
cd ~/下载
unzip m24e2f_real_fsm_preview_overlay.zip
cd m24e2f_real_fsm_preview_overlay
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2f_real_fsm_preview/run_offline_tests.sh" "$REPO"
```

This opt-in installer modifies native `main.cpp` (one include + a pre-DDS
non-loopback guard), native `CMakeLists.txt` (adds source/includes), and creates
one **NEW** simulation preview YAML, while saving copies of original main/CMake
in `m24e2f_real_fsm_preview/tools/backups/`. The existing simulation states,
original `config.yaml`, unitree SDK and motor outputs are not changed.

## Real native compilation (NOT a motor command)

```
cd "$REPO/deploy/robots/b2z1"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4 --target b2z1_ctrl
```

If CMake reports compile errors, paste the full first error. Offline source
checks cannot replace the native toolchain build on your development computer.

## Run preview only against MuJoCo and Z1 mock

First launch existing `unitree_mujoco --network lo` with B2+Z1 scene as before.
In another terminal start the E2D Mock Worker (use executable built by its
existing `run_offline_tests.sh` CMake recipe, e.g. from a permanent build dir):

```
E2D="$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge"
cmake -S "$E2D" -B /tmp/m24e2f_worker_build -DM24E2D_ENABLE_HARDWARE_SDK=OFF
cmake --build /tmp/m24e2f_worker_build -j4
/tmp/m24e2f_worker_build/m24e2d_z1_worker --socket /tmp/m24e2f_z1_mock.sock --backend mock
```

Main controller in a third terminal (only with MuJoCo loopback!):

```
export B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE
cd "$REPO/deploy/robots/b2z1/build"
./b2z1_ctrl --network=lo --config=config/config_m24e2f_real_input_preview_sim.yaml
```

Use `FixStand` then `RT + Y` to enter `InteractiveArmRealPreview`.
Keyboard i/k/j/h/u/n changes ONLY the IK **preview** target (1cm each; max 3cm
from entry). No Z1 motion is commanded, and no new B2 command is emitted.

## Not included

- Actual DDS reception callback stamp; packet arrival must be measured in the
  SDK2 subscription implementation rather than on FSM cache read.
- SDK mode TARGETs; currently blocked by E2e's conservative IPC client.
- B2 real motor command/handling when Z1 is detached from B2.
- Hardware watchdog response validation and bench stop qualification.

**This new module is a compile/inference preview, not the final physical FSM.**
