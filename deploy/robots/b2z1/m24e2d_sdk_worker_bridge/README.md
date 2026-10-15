# M2.4e2d — Dual-backend Z1 Worker (Mock default; SDK compile/link stage)

**Use**: first real Z1 SDK bridge code path for the established V1 Unix `SOCK_SEQPACKET` IPC and M2.4e2b trajectory planner. This add-only package leaves the existing B2 FSM, ONNX, MuJoCo, and all earlier overlays unchanged. Focus is developer-computer verification before the real-arm bench.

## Scope

- `src/z1_worker_dual.cpp`: preserves the earlier V1 IPC framing and Mock tests, and offers a separately gated `sdk` branch.
- `src/z1_sdk_bridge.cpp`, `include/z1_sdk_bridge.h`: based on the M2.4e2b SDK bridge, now exposes **measured initial feedback**, not hardcoded default angles, for `Planner::reset()`. Sampling timestamp is **not a proven UDP receive timestamp**.
- Backend uses SDK `sendRecv()` TX+RX only from a single worker control thread after the vendor startup thread stops. `z1_ctrl` is still the official Z1 control *service*, not a competing independent motion publisher.
- Initial SDK mode transition is `PASSIVE -> LOWCMD` via official API; no `backToStart()` or JOINTCTRL transition. This transition itself requires motor communication and may move/energize motors: **never consider SDK startup read-only**.
- Mock testing uses existing local fixtures (`m24e2a`, `m24e2b`, `m24e2c`) and adds CLI guards.

## SAFE DEFAULT: test without any robot / Z1 SDK

```bash
bash /path/to/repo/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/run_offline_tests.sh
```

`M24E2D_ENABLE_HARDWARE_SDK=OFF` (the default): the binary is not linked with or allowed to construct Z1 SDK. The 13 tests are software/Mock tests only.

## Developer PC SDK build/link check — DOES NOT execute

```bash
bash /path/to/repo/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/run_sdk_linkcheck.sh /path/to/repo
```

This explicitly compiles/links the SDK-enabled branch against `deploy/thirdparty/z1_sdk`. It **does not run the executable**. Vendor includes are treated as system headers, while our source uses `-Wall -Wextra -Wpedantic -Werror`.

**The SDK build/link check is not proof of safe physical arm operation.** The runtime physical activation requires separate operator action and is NOT part of this milestone. Do not start the SDK-enabled program on the returned real Z1 before confirming bench restraints, correct power/fuse, independently operable E-stop, controlled transition and power-loss behavior. Software watchdog holds are not certified physical stops.

## Important limitations before real movement

1. `Planner::reset()` checks complete real startup feedback; SDK getter sampling can still re-read cached data. There is no independently verified UDP RX provenance.
2. 150 ms target TTL, 0.4 rad/s and 0.8 rad/s² per-joint fixture are **unqualified bench values**, not safety-certified hardware limits. Joint limits are taken from previous development fixtures and must be checked against the actual Z1 and mount.
3. The SDK worker loop uses 2 ms timed polling, not a proven real-time 500 Hz scheduler. Measure worst-case `sendRecv()` blocking, jitter and deadline misses **before live arm motion**.
4. On SDK fault, stopping `sendRecv()` or destructing the SDK is **not a verified physical safe stop**. Qualify emergency hold / motor stop behavior and external E-stop first. Real `GET` reports `SDK_FAULT_LATCHED` instead of returning last data as valid after latched fault.
5. `z1_ctrl` service start and physical config.xml path require control-PC verification after software transfer. Do not run a separate `z1_ctrl k` while the SDK Worker owns commands.
6. No B2 real FSM/799D real feedback input integration in this package. That is the next milestone; retain the original `JointCtrlMockSim` untouched.

## Build outputs

- OFF: `m24e2d_z1_worker`, Mock-only, no vendor lib.
- ON: same executable with SDK code linked; **compile/link test only** in this milestone.

Please paste the two check outputs. Any SDK build/link error can be fixed in this add-only directory without touching B2 control or the official SDK.
