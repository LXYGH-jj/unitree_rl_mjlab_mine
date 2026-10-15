# M2.4e2a — Z1 independent worker IPC, MOCK-ONLY development slice

This is a **first small code deliverable** for the agreed ALORE-style architecture. It is deliberately *not yet real Z1 control*. It adds one directory only, and does not modify the already tested B2/MuJoCo FSM, CMake, ONNX, 799-D policy, or the existing JOINTCTRL/LOWCMD hardware guards.

## Verified scope

- Standalone C++17 executable, only Linux `AF_UNIX/SOCK_SEQPACKET`; no ROS2 / SDK headers or binary, DDS, UDP, B2 or Z1 IO.
- Request/reply protocol V1 with `HELLO`, `TARGET` and `GET` messages. Explicit mock provenance `V1 STATE MOCK` to prevent mistaking mock feedback for real Z1.
- Single connected client, Linux same-UID check, owner-only socket creation, strict sequence, sender-side monotonic age checks, six-joint finite/range checks, command TTL 150ms. A mock model slews joint positions for test coverage. Not motor physics.
- Read-only CLI (no physical arm effect), synthetic tests including watchdog, target movement, replay and out-of-limits rejection.
- No SDK source available means future real feedback timestamps will be *upper-layer observation timestamps*, not packet-origin evidence. Don't mislabel them.

## Development PC install (repo commit f2a3567 or later)

```bash
cd ~/下载
unzip m24e2a_z1_worker_ipc_overlay.zip
cd m24e2a_z1_worker_ipc_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2a_z1_worker_ipc/run_offline_tests.sh"
```

Build manually if desired:

```bash
SRC="$REPO/deploy/robots/b2z1/m24e2a_z1_worker_ipc"
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$SRC/build" -j2
```

**Protocol example** (no real SDK):

`V1 HELLO` -> `V1 HELLO_ACK MOCK_ONLY NO_SDK_NO_DDS_NO_MOTOR_IO`

`V1 TARGET seq sender_monotonic_ns enable q0 q1 q2 q3 q4 q5` -> `V1 ACK seq ACCEPTED|DISARMED`; values in radians, sequence strictly increasing. `enable=0` requests mock HOLD.

`V1 GET req_seq` -> `V1 STATE MOCK req_seq local_monotonic_ns last_target_seq target_age_ms TRACK|HOLD q0..q5 dq0..dq5`.

Mock response is always labeled `MOCK`; **never** use its values to authorize real Z1 commands. The transport interface is meant for a later dedicated B2 FSM reader using a nonblocking IO thread (not in the 200Hz motor control callback). The chosen V1 line format is a development fixture, subject to versioning once a production feedback transport is added.

## Next work, once this passes

1. Implement a separate SDK adapter in a further overlay, with a **single SDK communication owner** based on the official `z1_sdk/examples/lowcmd_development.cpp` and ALORE's `z1_control.py`. Bench qualification of Z1 electrical repair, e-stop, soft-start and fault stops is a separate gate, **not done by this package**.
2. Implement B2+Z1 `FeedbackSnapshot` combining real B2 DDS callback and a separate Z1 worker feedback, never reading Z1 from B2 motor_state[12:18]. Preserve all real-mode guards and keep the existing `State_InteractiveArmIK` sim-only.
3. Only after controlled bench checks and approved safety tests should real LOWCMD be enabled. Official `z1_ctrl` is a server, but there must be no second competing *SDK command client*.

## Git

```bash
cd "$REPO"
git add -- deploy/robots/b2z1/m24e2a_z1_worker_ipc/
git diff --cached --check
git diff --cached --stat
```

Do not `git add -A`, and do not stage unrelated local backup files.
