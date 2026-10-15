# M2.4d6 — B2 Walk + Z1 IK Simultaneous Motion Verification

**Read-only validation artifact. Not a controller patch.** It does not modify `b2z1_ctrl`, CMake, YAML, ONNX, SDK, ROS, DDS, or any motor interface. Base Git commit: `a8b4f5a`, branch `feature/b2z1-interactive-deploy`. This does not enable physical Z1 JOINTCTRL (pre-DDS real-output lock remains).

## Why this is needed

Existing M2.4d5 code already processes `i/k/j/h/u/n` during the bounded `w` forward walk. Earlier experimental logs demonstrated B2 **walking first** and Z1 IK **after stopping**; both activities occurring in one log is not evidence of simultaneous mobile manipulation. This script uses controller timestamps to demand overlap, without weakening any safety threshold.

## Offline installation

```bash
cd ~/下载
unzip m24d6_simultaneous_validation_overlay.zip
cd m24d6_simultaneous_validation_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24d6_simultaneous_validation/run_offline_tests.sh"
```

Installer **only adds** `deploy/robots/b2z1/m24d6_simultaneous_validation/`, does not stage or commit any files. The user's existing untracked backups and deleted document remain untouched. No new C++ build is required.

## Experiment — only an isolated MuJoCo loopback on the PC

1. Ensure no physical B2/Z1 robot, robot bridge, or shared DDS network can receive any simulated commands. Start the previous known-good B2+Z1 MuJoCo scene on `lo` (same M2.4d5 test environment).
2. In the control terminal, with normal stdin keyboard focus:

```bash
cd "$REPO/deploy/robots/b2z1/build"
export B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE
set -o pipefail
./b2z1_ctrl --network=lo \
  --config=config/config_interactive_jointctrl_mock_m24d5_sim.yaml \
  2>&1 | tee "$HOME/m24d6_walk_arm_$(date +%Y%m%d_%H%M%S).log"
```

3. Press `f` to enter FixStand. Allow stabilization. Press `g` to enter JointCtrlMockSim. Wait for the state warmup (>=2 seconds). Press `w`, **release it**, then immediately press `j` once (Z1 target +1cm Y) **before** the `max forward 2000ms reached` message. Watch the robot in MuJoCo. Do not hold both keys; the keyboard implementation processes one key at a time. During the walk the Z1 IK target should appear in the log. Walk auto-stops after 2 seconds. You can press `h` after the walk to restore Y. Use `f` to leave the state.
4. Use Ctrl-C after clean exit to stop the controller and close the log; Ctrl-C stops the process, **not** a physical emergency stop (the setup must remain simulated).

Analyze your saved log (substitute the real filename):

```bash
python3 "$REPO/deploy/robots/b2z1/m24d6_simultaneous_validation/tools/analyze_sim_overlap.py" \
  "$HOME/m24d6_walk_arm_YYYYMMDD_HHMMSS.log" \
  --json-out "$HOME/m24d6_report.json"
```

Or pick the newest matching log with the following shell sequence (always check the printed filename):

```bash
LOG=$(ls -t "$HOME"/m24d6_walk_arm_*.log | head -1)
echo "Checking log: $LOG"
python3 "$REPO/deploy/robots/b2z1/m24d6_simultaneous_validation/tools/analyze_sim_overlap.py" "$LOG"
```

## Output interpretation

- `PASS_OUTPUT_EVIDENCE`: one **accepted** IK goal lies within the B2 walking interval; active positive-vx diagnostics and increasing Mock gate counts are observed; joint 1 commanded position changed >0.01 rad in 1Hz sampled telemetry. This is **controller-log evidence**, not a world-frame EE-tracking guarantee.
- `PASS_COMMAND_PATH_ONLY`: IK, walking and Mock gate overlap are logged but the 1Hz joint-1 target did not show sufficient difference. Other joints may have moved; to prove it further, inspect MuJoCo video or add six-joint telemetry as a separate opt-in change. This is not a full motion-following PASS.
- `NO_SIMULTANEOUS_EVIDENCE`: IK goals occur only after stopping, or missing positive-velocity/gate data.
- `FAULT`: detected explicit safety fault / arm lock during the examined state. Stop testing; do not relax safeguards.
- `INCOMPLETE`: controller output did not include the auto-stop, e.g. capture ended too early.

**Known limitations:** timestamps are PC process wall-clock; the original M2.4d5 log exposes only q0 at ~1Hz. Joint-space deviation ~0.05 rad and EE error ~0.05m are known in this simulation and are recorded, not modified. This experiment does NOT demonstrate reliable world-frame EE tracking while locomoting, actual B2 base displacement, safety of physical Z1, nor independent genuine UDP RX evidence. The legacy vendor output path remains locked.

## Commit only after verifying

```bash
cd "$REPO"
git add -- deploy/robots/b2z1/m24d6_simultaneous_validation/
git diff --cached --check && git diff --cached --stat
git commit -m "Add M2.4d6 read-only walking and arm overlap verifier"
git push origin feature/b2z1-interactive-deploy
```

Never `git add -A` in the current dirty workspace. Do not add generated `*.log`, virtual environments, backups or the unrelated deleted document.
