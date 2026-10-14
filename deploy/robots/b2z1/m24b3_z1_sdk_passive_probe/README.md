# M2.4b3 — Independent Z1 official SDK feedback diagnostic candidate

Target: `unitree_rl_mjlab_mine`, branch `feature/b2z1-interactive-deploy`.

**M2.4b2 success is established; this adds the next independent diagnostic tool.**
This package is installed beside M2.4b2, not integrated into B2 or the FSM.

## Warning: this is NOT a passive network observer
`UNITREE_ARM::unitreeArm::sendRecv()` **both sends an SDK command packet and receives feedback**. Not calling `setFsm`, `startTrack`, or `setArmCmd` does not guarantee the robot is unaffected; SDK initialization and traffic semantics require hardware-level validation. The sample program deliberately does NOT automatically launch `z1_ctrl`, does NOT request any arm control mode, and does NOT send a target position. It is NOT approved for unsupervised real-hardware operation and MUST NOT be used following a fuse failure before physical electrical/drive inspection.

Feedback copied from the SDK `lowstate` getter is not independent proof of a *new UDP packet*. This module NEVER sets `confirmed_new_packet=true` and is not eligible to feed safety-critical 799-observation control. Inspect and instrument the actual receive path or use an independently validated packet capture before implementing M2.4c.

## Offline procedure (no robot I/O)
```
cd ~/下载/m24b3_z1_sdk_passive_probe
bash run_offline_tests.sh
bash build_sdk_monitor.sh /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
bash install.sh /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
```
`build_sdk_monitor.sh` compiles/links the Z1 SDK on the workstation, then invokes only `--help`: **it does not construct the SDK object**. All other hardware I/O requires an explicit runtime opt-in AND physical inspection; do not enable it as part of this step.

## SDK bench preparation, separately planned
Before any physical connection/powered test: qualified electrical inspection of fuse/supply/cabling, verified E-stop and mechanical support, no competing `z1_ros2`, `z1_ctrl k`, or other SDK client. The independent `z1_ctrl` process must be the intended SDK-mode official controller and its FSM state confirmed PASSIVE. Next hardware stage to be reviewed separately; the gated candidate currently includes `--bench-sdk-passive-observation` but **do not run it until an attended bench procedure has been accepted**.

## Scope
- No changes to `main.cpp`, `FSMState.h`, CMakeLists or B2 DDS/SDK2 path.
- No changes to M2.4b1 LOWCMD bridge.
- New telemetry candidate uses a single `arm.sendRecv()` caller, not the SDK background thread plus another loop.
- Sample is finite (20 x 100 ms) and immediately rejects abnormal motor, SDK and controller state flags.
- Physical state is NOT actively brought into PASSIVE by this tool. It checks after the first SDK send/receive, so it cannot be treated as proof that the first packet was non-actuating.
