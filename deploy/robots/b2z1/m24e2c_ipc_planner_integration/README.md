# M2.4e2c — IPC V1 + M2.4e2b Planner integration (MOCK ONLY)

**Scope:** This stage **actually links** the M2.4e2a Unix `SOCK_SEQPACKET` IPC Worker to the real M2.4e2b `Planner` C++ class, using a **synthetic perfect-servo plant**. No Z1 vendor SDK is linked, constructed, loaded or called. This is **NOT real LOWCMD hardware operation**. The `--backend sdk` argument explicitly refuses to run. Default compile includes `-Wall -Wextra -Wpedantic -Werror` for our own code.

- IPC V1 wire grammar unchanged: `V1 HELLO`, `V1 TARGET seq monotonic_ns enable q1..q6`, `V1 GET request_id`. Maintains M2.4e2a responses and all four regression tests.
- `V1 DIAG` is an optional, additive command (`V1 DIAG MOCK_ONLY OK|FAULT code`) to inspect latched planner faults without modifying V1 STATE shape.
- The mock-specific device-feedback booleans describe synthetic data ONLY and do not qualify hardware feedback or SDK RX provenance.
- Planner enforces offline fixture limits, command freshness, sequence order, velocity/acceleration and target TTL. Target replay or invalid values latch the session fault; reconnect to establish a new session. The synthetic plant copies the planner setpoint as measured state for deterministic unit testing. This is **not** an accurate dynamic motor simulation.
- V1's `MOCK` status remains explicit to prevent accidental substitution as real Z1 joint feedback.
- The simulated fault "hold" response is only an interface behaviour. It does not imply a physical E-stop or guarantee real LOWCMD stop/hold.

## Install on developer PC

```bash
cd ~/下载
unzip m24e2c_ipc_planner_integration_overlay.zip
cd m24e2c_ipc_planner_integration_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2c_ipc_planner_integration/run_offline_tests.sh"
```

Test suite builds C++ with GCC and runs **9 tests**: 5 newly integrated, plus 4 unchanged M2.4e2a IPC regression tests. It does not require a robot, Z1 SDK linking, DDS or network access.

## Important engineering gap before genuine SDK mode

**Do not flip a macro to enable motors yet.** `SdkBridge::start()` in M2.4e2b verifies and saves a local SDK start sample, but does not expose an immutable initial `Feedback` to its caller. The hardware planner must be bootstrapped using **actual measured Z1 q/dq/error/mode**, never zeros, an ALORE default pose or the simulated state. In the next integration stage, add an explicit startup-feedback API, synchronize SDK lifecycle with a **single `sendRecv` owner**, validate LOWCMD gain settings and test exit/fault behavior on a fixed bench after basic power/E-stop checks.

Other prerequisites: implement a distinct `SDK_GETTER_UNVERIFIED` feedback status rather than `MOCK`, validate target and measured state timeliness, and correctly select real Z1 q/dq for the 799D policy. Do not mistake getter `read_at` for official UDP reception proof. Retain all previous JOINTCTRL hardware refusals.

## Status

- M2.4e2a IPC MOCK tests — previously passed on user PC.
- M2.4e2b planner tests and Z1 SDK header syntax checks — previously passed on user PC, vendor warnings isolated via `-isystem`.
- M2.4e2c IPC+planner integration — **9 offline tests passed in development container**; pending user PC run.
- Real Z1 SDK LOWCMD Worker — **not yet implemented/verified as executable**.
- Real Z1 hardware commands — **blocked**.

Do not stage unrelated files:
```bash
git add -- deploy/robots/b2z1/m24e2c_ipc_planner_integration/
git diff --cached --check
```
