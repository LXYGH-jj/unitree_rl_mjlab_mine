# M2.4e2e — Z1 IPC client + real B2/Z1 observation input (first integration step)

**Scope (not hardware-qualified):** add-only independent code. This is an IPC client,
read-only background state poller, and an adapter to the repo's original `adaptFeedback()`
/ `ObservationBuilder` 799-D layout. It deliberately does **NOT** register or enable
`InteractiveArmReal`, modify the native B2 FSM, send real B2 `LowCmd`, or send
Z1 SDK TARGET packets. Real Z1 actuation remains a separate bench validation step.

## What is implemented

- AF_UNIX/SOCK_SEQPACKET V1 HELLO/GET/TARGET client, single socket owner.
- Strict Mock-vs-SDK HELLO and response-mode checks; rejects malformed, nonfinite,
  oversized, wrong request ID, and old IPC response timestamp.
- `ReadOnlyPoller` reads Worker feedback at approximately 50 Hz in a background
  thread and makes a thread-safe copy available without blocking the B2 FSM.
  On any IPC error, it invalidates state and **does not autoreconnect**.
- TARGET is implemented only for Mock. SDK movement from this new client is
  deliberately refused until E-stop, gains, watchdog and stop behaviors are
  qualified on the repaired Z1 bench. The M2.4e2d Worker remains the sole SDK owner.
- An explicitly opted-in Mock J5 jog mode now reuses the SAME IPC owner thread
  for 20ms GET/TARGET refresh; other 5 joints retain their first Mock GET
  values, J5 offset is capped at +/-0.030 rad, and SDK mode cannot enable it.
  Default ReadOnlyPoller remains GET-only. See docs/mock_j5_fsm_stage1.md.
- `makeLiveInput()` combines REAL B2 leg/IMU feedback + Z1 IPC q/dq and calls
  **existing** `interactive_b2z1::adaptFeedback()`; it preserves the 12+6 joint
  order, nominal training EE coordinates and B2/Z1 age/skew checks.
- Unit test constructs a full 799-dimensional observation via the existing
  `ObservationBuilder` when the real repo path is supplied.

## Caveats that block direct actuation

0. **SDK mode is not physically read-only:** opening a session with an E2D Worker
   running `--backend sdk` can cause its SDK startup and LOWCMD mode transition,
   even if this client only requests `GET`. Do **not** connect a real arm
   until the separate bench procedure/stop behavior is approved.
1. **B2 timestamp:** `B2State.packet_received_at` must come from the actual
   DDS arrival callback's `steady_clock::now()`. Stamping the FSM's cache read
   time would hide an old frame and invalidates freshness checks. This
   repo's production integration has NOT been modified yet to provide it.
2. **Z1 timestamp:** Worker's `V1 STATE` timestamp is a fresh response stamp,
   **not proof of a fresh vendor SDK UDP receive**. `ArmState.ipc_received_at`
   means only the client received an IPC reply. E2e never labels it hardware RX.
3. **Old Z1 bridge:** the pre-existing production `config/config.yaml` still
   defaults to `z1_sdk.enabled: true` and `z1_diagnostic.enabled: true`.
   The eventual Real FSM config must set **both to false** and have only the
   separate Worker own the SDK. Never run both SDK clients against one Z1.
4. **Installation geometry:** Z1 is now bench-mounted, so do not treat
   `[0.03,0,0.08]` as the bench extrinsics or qualify mobile manipulation yet.
5. **No real motor command route:** existing simulator FSM reads Z1 from
   `motor_state()[12+i]`, which is NOT a valid real-Z1 feedback source.
   Real FSM must be new and separate. The B2 motor output must use B2 slots
   0..11 only; do not enable existing simulation-only FSM on non-loopback.

## Install on developer PC

```bash
cd ~/下载
unzip m24e2e_live_input_overlay.zip
cd m24e2e_live_input_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2e_live_input/run_offline_tests.sh" "$REPO"
```

Test script builds new code against **the actual existing** `interactive_observation.cpp`
and `interactive_feedback_adapter.cpp`, plus the current E2D mock worker. It
DOES NOT initialize vendor SDK/DDS or touch the robot. If you run the test
without REPO, it uses a minimal local API fixture and is a weaker check.

## Intended next native FSM integration

1. Add an SDK2 LowState **DDS callback arrival timestamp** to real B2 state
   snapshot; validate it rather than stamp the subsequent read.
2. Instantiate a `ReadOnlyPoller` for the Z1 Worker; `latest()` is copy-only.
3. Collect B2 first 12 motor joints, quaternion and gyroscope from the real
   SDK2 LowState, and pass proper arrival time to `makeLiveInput()`.
4. Call the existing `InteractivePipeline::step()` with the adapted input,
   feeding 799-D observation. Verify 18 action semantics before actuation.
5. Add separate real FSM, IK, and gated B2/Z1 target commands after the bench
   validates physical stop semantics. Keep original sim FSM unchanged.

This split is deliberate: the source already has a working Interactive sim FSM,
but it is **strictly SIM_ONLY** with `motor_state()[12..17]`. Forcing it to
accept real hardware is unsafe and cannot be justified by E2D link success.
