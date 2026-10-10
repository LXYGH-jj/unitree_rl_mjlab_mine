# M1 — Interactive 799d observation parity (offline only)

This standalone C++17 module reconstructs the exact `lowlevel_core.py` observation layout **without DDS, SDK2, Z1 SDK, FSM, or motor commands**.

Input full 18 DOFs: **FR FL RR RL Z1[1..6]**. SDK2 motor-to-policy leg mapping is `[3,4,5,0,1,2,9,10,11,6,7,8]` (confirmed by user). Z1 state is external; never pretend that B2 `lowstate` contains the last six joints on hardware.

Observation: `71 current | 18 priv | 10*71 previous history`, size 799. Privileged placeholder has `priv[5]=1`; initial history is zeros; history advances **after** constructing output. The last 12 actor actions enter the *next* observation. This class **does not** run an actor, calculate joint targets or safety checks, or send commands.

## Build

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
cmake -S deploy/interactive_b2z1 -B deploy/interactive_b2z1/build
cmake --build deploy/interactive_b2z1/build -j2
ctest --test-dir deploy/interactive_b2z1/build --output-on-failure
```

## Verify Python/C++ observation parity (requires `numpy`, `pyyaml`)

```bash
python deploy/interactive_b2z1/tests/test_obs_parity.py \
  --fixture deploy/interactive_b2z1/build/interactive_obs_fixture
```

This checks 5 sequential samples, varying commands, gait stop/restart, history, previous actor output including clip at 100, and baseline config values. Expect `M1 OBS PARITY: PASS` with max absolute diff <= 1e-5.

## Restrictions and follow-up

- `ObservationConfig::baseline()` intentionally hardcodes **only** baseline observation constants; parity test fails if Python YAML parameters diverge. Later read the same YAML in the deployment integration or centralize config.
- End-effector observation is **desired EE goal in Interactive's nominal arm-base frame**, not measured TCP position. IK/FK and actual arm mounting `[0.03,0,0.08]` are separate modules.
- This builder does not inspect feedback timestamps, handle stale data or enforce command safety; integrator must reject invalid/stale feedback and apply independent B2/Z1 safety logic.
- Python parity here validates **only observation construction**. ONNX vs TorchScript and deployed real-time loop remain separate tests.
- No existing C++ controller file has been modified. Do not wire this into live `State_RLBase` until test passes and mapping/feedback freshness is verified.
