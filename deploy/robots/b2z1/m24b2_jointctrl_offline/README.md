# M2.4b2 — Z1 official JOINTCTRL offline foundation

**Target repository:** `LXYGH-jj/unitree_rl_mjlab_mine`, branch `feature/b2z1-interactive-deploy`.

**Purpose:** preserve the known-working B2 SDK2 communication and the M2.4b1 LOWCMD backup, and establish an independent JOINTCTRL-based Z1 controller foundation with an offline safety core and SDK API compilation check.

## Important restrictions

- **OFFLINE ONLY: does not connect to or control real Z1 or B2.** The vendor SDK adapter explicitly throws on `connectReadOnly()`, `enterJointCtrl()`, `copyFeedbackUnverified()`, and `sendJointCtrlCommand()`.
- No `z1_ctrl` processes or DDS channels are launched; no motion, mode switch, SDK library loading, socket creation, or motor command.
- Tests' `q_min=-2, q_max=2, max_velocity=0.5` are **fabricated for unit tests**. Do not copy them into a real robot's joint limits.
- M2.4b2 is NOT a certified actuator safety system. A software stop without a verified hardware transition may leave the last servo setpoint active.
- A true new-packet reception event is not exposed by SDK `lowstate` getters; `isDisConnect` is a health flag only. The real adapter must independently verify feedback freshness before authorizing control.
- JOINTCTRL can still generate dangerous joint motion if fed unsafe commands; vendor control mode does not cure collision, electrical, or tracking hazards.

## Install

```bash
cd ~/下载
unzip m24b2_z1_jointctrl_offline.zip
cd m24b2_z1_jointctrl_offline
bash run_offline_tests.sh /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
bash install.sh /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
```

Only adds `deploy/robots/b2z1/m24b2_jointctrl_offline/`, refuses overwriting a previous installation. No edits to original files or existing build.

## Module design

- `include/z1_jointctrl_safety.h`, `src/z1_jointctrl_safety.cpp`: pure C++17 gate; explicit mock joint soft limits, packet confirmation, feedback age and error-state checks, acknowledgement, bounded position increments, velocity feedforward, tracking-error latch, missed-loop latch. No hardware I/O.
- `include/z1_jointctrl_sdk_candidate.h`, `src/z1_jointctrl_sdk_candidate.cpp`: vendor SDK `startTrack(JOINTCTRL)` and `setArmCmd(q,dq)` API signature checks. All runtime entry points intentionally refuse hardware I/O.
- `tests/test_jointctrl_safety.cpp`: offline healthy, stale feedback, duplicate/non-confirmed packets, motor faults, joint limits, tracking and timeout tests.
- `docs/next_steps.md`: independent real SDK evidence collection and integration gates.

## Why not edit the old z1_sdk_controller.cpp?

M2.4b1's original controller is LOWCMD-based and currently intentionally locked; keep it as a future research baseline. Replacing its implementation in place could accidentally reactivate LOWCMD or damage the known-good MuJoCo baseline.

## Official references

- https://github.com/unitreerobotics/z1_sdk/blob/master/examples/highcmd_development.cpp
- https://github.com/unitreerobotics/z1_sdk/blob/master/examples/lowcmd_development.cpp

The official high-level example runs the vendor background `sendRecvThread` and calls `startTrack(JOINTCTRL)`, `setArmCmd(q, dq)`. Do **not** additionally run a custom `sendRecv()` loop on the same SDK object.
