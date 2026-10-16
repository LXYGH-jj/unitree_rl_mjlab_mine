# Stage 1: B2 native FSM -> one IPC thread -> Mock Z1 J5

## Scope

SIMULATION ONLY. No Z1 SDK TARGET, no real B2 motor output. The existing
`m24e2d_z1_worker` hardware SDK, Planner, gains and physical bench controller
have NOT been changed.

The existing `ReadOnlyPoller` is read-only by default. A new **explicit**
`mock_j5_control=true` option allows ONLY the same poller thread to refresh
Mock Z1 TARGETs (20 ms period). No second IPC socket/thread sends commands.

`Session::target()` still raises `SDK TARGET disabled pending supervised
bench qualification` when connected to real SDK mode. Default E2g real
configuration still refuses `enable_z1_sdk_targets: true`.

The simulated keyboard target is an offset relative to first Mock GET:
only joint 5 (zero-based index 4), offset within +/-0.030 rad.
Other joint targets remain at the initial Mock state; they cannot be
commanded through this simulated jog feature.

## First run (offline; no DDS, no Unitree SDK, no motors)

```bash
export REPO=/home/liu/deploy_ws/b2z1/mine_ws/unitree_rl_mjlab
cd "$REPO"
git pull --ff-only origin feature/b2z1-interactive-deploy
bash deploy/robots/b2z1/m24e2e_live_input/run_offline_tests.sh "$REPO"
bash deploy/robots/b2z1/m24e2g_real_control/run_offline_tests.sh
cmake --build deploy/robots/b2z1/build -j4 --target b2z1_ctrl
```

Expect `M2.4e2e MOCK J5 COMMAND PASS` (plus previous tests).

## MuJoCo + separate Mock Z1 Worker (no Z1 motor or SDK)

After the offline tests pass:

1. Start the existing previously verified MuJoCo B2 scene on loopback `lo`.
2. Start a **Mock-only** Worker from a build with
   `-DM24E2D_ENABLE_HARDWARE_SDK=OFF`:
   ```bash
   E2D="$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge"
   cmake -S "$E2D" -B /tmp/b2z1_mock_worker_build \
     -DM24E2D_ENABLE_HARDWARE_SDK=OFF
   cmake --build /tmp/b2z1_mock_worker_build -j4
   /tmp/b2z1_mock_worker_build/m24e2d_z1_worker \
     --socket /tmp/m24e2g_mock_joint_jog.sock --backend mock
   ```
3. Start **only on loopback**:
   ```bash
   export B2Z1_REAL_DRYRUN_ACK=SIMULATION_ONLY
   cd "$REPO/deploy/robots/b2z1/build"
   ./b2z1_ctrl --network=lo \
     --config=config/config_m24e2g_mock_joint_jog_sim.yaml
   ```
4. In controller terminal enter FixStand using `f`, and then the existing
   InteractiveArmReal state using `g`. In that state `+`/`-` jog
   Mock J5 by 0.005 rad per detected key event, `0` resets the **Mock**
   relative goal, and `p` asks for Passive.

**Important:** Mock Z1 feedback is a separate synthetic simulation, NOT the
Z1 body in MuJoCo. The B2 simulation does not visually move its Z1 link
because Mock TARGETs are not published to its arm joints. Logs/GET are
the expected observation for Stage 1.

The E2g controller requires its configured ONNX model and a valid B2
FixStand pose before entering the state. If these checks fail, fix the
simulation/model configuration; do NOT try a real network or turn on real
B2 output to bypass them.

## Deliberately not completed

- C++ physical Z1 TARGET release and vendor UDP freshness certification.
- B2 physical FixStand + Z1 movement: requires independent B2 motor/mode
  validation and reviewed stop/fault handoff.
- Full 6-DOF IK motion, arm policy action mapping, physical leg-arm coordination.

Never run this simulation YAML on physical B2 or Z1. `SDK` mode
on Worker can drive Z1 even while FSM only GETs state.
