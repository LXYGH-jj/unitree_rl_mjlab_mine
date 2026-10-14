# M2.4d4: Z1 transport contract + cooperative owner (MOCK ONLY)

Baseline: `LXYGH-jj/unitree_rl_mjlab_mine` commit **`94bc226`** on `feature/b2z1-interactive-deploy`.

This is **not a Z1 hardware driver**, and does not change the official library, main FSM, B2 SDK2 or MuJoCo. It is a stand-alone C++ interface and safety-gate integration test for the next FSM mock workstream.

## Safety and scope

- `IZ1Transport` abstracts a feedback queue, command sink and a logical disarm operation.
- `MockZ1Transport` only generates **synthetic in-memory** feedback and records bounded commands in memory. Its `RxEvidence` booleans are synthetic fixtures, **NOT independently verified physical UDP receiver evidence**.
- `MockOnlyRuntime` rejects any transport other than `TransportKind::SyntheticMock`. Its `authorizeForOfflineTest` passes synthetic approvals into the already audited M2.4d1 and M2.4c3 gates. It **MUST NOT** be linked to production firmware or a physical command sink.
- `VendorDisabledTransport` has **no official SDK import**, cannot start JOINTCTRL or perform IO, returns no feedback, and throws on every command emission. It cannot be used in `MockOnlyRuntime`.
- `CooperativeOwnerLease` uses Linux `flock(LOCK_EX|LOCK_NB)` to prevent a cooperating process from acquiring the same lock path simultaneously. It **cannot** enforce exclusivity against the vendor `z1_ctrl`, other SDK clients, unrelated ports, or an uncooperative process. It does not check the robot's physical communication owner. Locks may persist through forked children inheriting the descriptor and are not a hardware interlock.
- `disarm()` only prevents additional software emissions; it **does not** guarantee controlled stopping, gravity compensation or E-stop. The prior Z1 fuse failure remains unresolved and requires independent electrical inspection before hardware activation.
- No vendor SDK calls, network sockets, DDS, real JOINTCTRL, hardware or motors occur. This module is intentionally **NOT** included in production `b2z1_ctrl`'s CMake.

## Architecture

```
 SyntheticFrame --> MockZ1Transport --> MockOnlyRuntime.pumpOne()
       --> M2.4d1 TransportSession --> M2.4d1 RxValidator
                                     --> M2.4c3 JointCtrlSession --> SafetyCore
                                                              --> Mock sink ONLY
 VendorDisabledTransport -----------------------------X (always locked)
```

## Install

```
cd ~/下载
unzip m24d4_z1_transport_contract_overlay.zip
cd m24d4_z1_transport_contract_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh "$REPO"
```

Installer requires M2.4b2/c3/d1/d2/d3 and the existing pre-DDS JOINTCTRL hardware refusal guard. It only creates `deploy/robots/b2z1/m24d4_z1_transport_contract`; no existing source or CMake modification. A second install refuses to overwrite.

## Offline verification

The test builds a separate executable with the real M2.4b2/c3/d1 files and tests: 3 synthetic frames, permission gate, bounded `q` step, stale timeout, overtemperature, nonmonotonic frame, injected sink failure, vendor refusal, same-process and cross-process **cooperative** lease conflict. No SDK or robot connection required. Python fixtures verify installer non-destructive behavior. These do not establish physical control safety.

```
# after initial test, optional standalone check
cmake -S "$REPO/deploy/robots/b2z1/m24d4_z1_transport_contract" -B /tmp/m24d4_manual_build
cmake --build /tmp/m24d4_manual_build -j2
ctest --test-dir /tmp/m24d4_manual_build --output-on-failure
```

## Next workstream

M2.4d5: wire the MOCK-ONLY contract into a separate MuJoCo FSM integration test without bypassing or weakening the original `z1_jointctrl.enabled: true` pre-DDS refusal. Vendor RX remains blocked until the SDK delivers demonstrably atomic receive events and matching decoded snapshots through a supported mechanism. Interprocess locking by itself cannot establish hardware control ownership. Independently validate actual motor stop behavior and electrical installation before any real Z1 command path.
