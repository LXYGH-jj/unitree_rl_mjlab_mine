# M2.4e2g — Real FSM candidate, DDS reception timestamp, keyboard-only

This is the next **production-code engineering step**, not a qualified hardware deploy.

## What changed

- `StampedB2LowState` subclasses Unitree SDK2's `go2::subscription::LowState` and overrides the SDK2 `post_communication()` callback, stamping each received B2 DDS `LowState` under `mutex_`. No vendor source modifications.
- New `State_InteractiveArmReal` registered with native FSM. Receives real B2 DDS 12-leg+IMU state and separate Z1 Worker GET feedback through existing E2e `ReadOnlyPoller`; calls existing 799-D `makeLiveInput`, actual 18-output ONNX, leg motor target generation, and Z1 FK/IK.
- B2 leg target code writes **only B2 joints 0..11** at 200 Hz and invokes original native `FSMState::post_run`. It is compile-ready but gated by YAML and a separate environment acknowledgment. **Not validated for physical B2**.
- Real FSM's IK target is explicitly **NOT SENT** to physical Z1 in this stage. The E2e client itself hard-blocks SDK TARGET; no watchdog or STOP semantics are inferred from SDK linking alone.
- Isolated real YAML has just `Passive`, `FixStand`, `SlowLieDown`, `InteractiveArmReal`. Array lengths are converted to **12 B2-only joints**, and previous vendor direct SDK controller and diagnostics remain disabled.
- Main program instantiates the derived DDS timestamping subscriber **only when** `interactive_real.enabled=true`. Existing MuJoCo and M2.4d6 control branches are unchanged.
- Keyboard-only mode constructs joystick events solely from keyboard, ignoring physical handset inputs; `f` FixStand, `g` Real, `p` Passive, `l` LieDown, `i/k/j/h/u/n` IK preview, `space` stops gait, `w` requests bounded forward movement (walking disabled by default).

## Install on developer PC

```bash
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
cd ~/下载
unzip m24e2g_real_control_overlay.zip
cd m24e2g_real_control_overlay
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2g_real_control/run_offline_tests.sh"
cmake -S "$REPO/deploy/robots/b2z1" -B "$REPO/deploy/robots/b2z1/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO/deploy/robots/b2z1/build" -j4 --target b2z1_ctrl
```

The installer backs up three changed production files under `m24e2g_real_control/tools/backups/` before editing them and adds a separate `config/config_m24e2g_real_candidate.yaml`. It never overwrites old MuJoCo/YAML branches or vendor SDK files.

## **DO NOT RUN the E2g real YAML on the robot yet**

1. `interactive_real.enable_b2_output: false` (default) **will refuse nonloopback startup** before DDS. The native FSM still commands motors in `Passive/FixStand`; thus merely disabling output in the new state is NOT enough to make running the whole native binary physically read-only.
2. `interactive_real.enable_z1_sdk_targets: false` is mandatory. No automatic SDK Z1 target output exists in E2g; it remains blocked by the E2e IPC client.
3. Z1 `GET`/IPC timestamps are not proof of new vendor UDP packets. No guaranteed safe behavior for vendor SDK fault, stop, mode transition or power loss has been shown yet.
4. Separate real **Z1 fixed-bench** acceptance comes before releasing SDK target output. First confirm vendor `z1_ctrl` config, repaired fuse/power condition, mechanical restraint, stop behavior, and actual joint feedback under controlled conditions.
5. B2 physical motion needs its own bench confirmation, real motor order/gains and explicit command recovery in faults. **Do not set either output flag or acknowledgment prematurely.**
6. The on-fault FSM fallback to FixStand is an engineering mechanism, not an E-stop and not proven on physical hardware.

No need to repeat MuJoCo actual movement testing. Please run the **native compilation** and provide the complete first compiler error if any. After that, target the Z1 fixed-bench acceptance and SDK-target enablement.
