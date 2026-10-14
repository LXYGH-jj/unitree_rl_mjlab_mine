# M2.4c3 — JOINTCTRL command-execution gate (offline-only until hardware prerequisites)

M2.4c2 already proved registered FSM, 799D ONNX, arm FK/IK and EE keyboard input on MuJoCo `lo`. M2.4c3 is a **different milestone**: actual 6-joint q/dq commands pass through one executable, velocity-limited, watchdog-protected `JointCtrlSession`, and are emitted into an injected mock `CommandSink`. This isn't another 799D shadow computation: the `CommandSink::emit()` boundary is called and its exact outputs tested. The injected sender is not the official Z1 SDK yet.

## What is now ready
- A production-shaped `JointCtrlSession` that takes six-joint IK targets and confirmed receiver-side Z1 feedback, checks explicit per-joint real limits, velocity bounds, tracking error, motor error/connection flags, freshness, deadlines and operator authorization; only then forwards a bounded `(q,qd)` pair to a single sink.
- Test sends **two distinct bounded joint commands** through a mock sender. Tests assert no output before authorization, no output after software disarm, stale/unproven packet rejection, motor-health fault rejection, timeout, and transmission exception latched stop.
- SDK API compile-only check for `startTrack(JOINTCTRL)`, `setArmCmd(q,qd)`, and SDK `sendRecvThread` existence. Does not instantiate the SDK, start controller process or use any network.
- CMake sources added to the existing b2z1 controller library without modifying M2.3, M2.4c2, or the working B2 SDK2 FSM/control code. Existing M2.4c2 real JOINTCTRL rejection **before DDS initialization** preserved.

## What is NOT ready / why a real motor cannot be enabled
M2.4b3 SDK lowstate getters and `isDisConnect` do **not** prove receipt of a fresh Z1 UDP packet. SafetyCore correctly rejects `confirmed_new_packet=false`. Therefore **do not set it true from local wall-clock, SDK getter polling or `sendRecv()` cycles**. Production `CommandSink` adapter to the official Z1 SDK is deliberately not connected. We also have no confirmation that the previous fuse blow-out's electrical cause was inspected or rectified. An SDK session isn't safe merely because C++ links.

Motor temperature interlock is **not yet implemented** in this execution gate and must be added before real actuation.

The official `z1_sdk` `highcmd_development` example uses one vendor `sendRecvThread` with `arm.startTrack(ArmFSMState::JOINTCTRL)` and `arm.setArmCmd(q,qd)`. For future real integration, use that SDK thread exclusively; **never start a custom `sendRecv()` loop concurrently**, and do not invoke `backToStart()` or automatic `PASSIVE` on the combined mounted robot without safety review. Device-level protective stop/support mechanism is not implemented in this module.

## Install on developer PC (NOT a robot test)
```
cd ~/下载/m24c3_jointctrl_runtime_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh "$REPO"
cmake -S "$REPO/deploy/robots/b2z1" -B "$REPO/deploy/robots/b2z1/build"
cmake --build "$REPO/deploy/robots/b2z1/build" -j2 --target b2z1_ctrl
```
Note: offline audit requires the installed new CMake source, so `run_offline_tests.sh` should be run **after** `--apply` (or run pre-install C++ test separately).

## Git handoff
Do not use broad `git add .`; keep backups, old debug builds, LOWCMD archived experiments, and any locally deleted tracked file out of this submission.
```
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
git status --short
git diff --check
git add -- deploy/robots/b2z1/CMakeLists.txt deploy/robots/b2z1/m24c3_jointctrl_runtime/
git --no-pager diff --cached --stat
# Include M2.4c2 main/CMake/FSM/YAML/source only after inspecting status and separately staging those exact files.
```
Build and verify in a fresh Git worktree before push, as with version 7aa7c08.

## Next gating milestone
1. Implement a genuine receiver-origin freshness proof (SDK receive UDP sequence or callback); audit its correctness independently.
2. Integrate official SDK mode/session ownership with documented stop semantics; no simultaneous vendor and custom sendRecv owners.
3. Verify joint/model calibration, real limits, B2/Z1 time synchronization, mounting clearance and operator E-stop.
4. Independently resolve previous fuse/electrical issue. Only then supervised independent Z1 bench motion; full mounted B2+Z1 later.
