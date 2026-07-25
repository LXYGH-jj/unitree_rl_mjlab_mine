# B2Z1 Deploy Notes

This directory contains the B2+Z1 deploy controller and the runtime assets
needed for both simulation and real-robot validation.

The controller source and binaries live in:

`deploy/robots/b2z1`

The Z1 runtime dependencies are bundled under:

`deploy/thirdparty/z1_sdk`

`deploy/thirdparty/z1_ctrl`

## 1. Copy the latest ONNX

Your training runner exports an ONNX alongside checkpoints. Copy or rename the chosen model to:

`deploy/robots/b2z1/config/policy/velocity/v0/exported/policy.onnx`

Example:

```bash
cp /path/to/your/exported_run.onnx \
  /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2z1/config/policy/velocity/v0/exported/policy.onnx
```

## 2. Runtime layout

For portable deployment, keep the following structure inside this repository:

```text
deploy/
  thirdparty/
    z1_sdk/
      include/
      lib/
        libZ1_SDK_x86_64.so
    z1_ctrl/
      z1_ctrl
      lib/
        libZ1_x86_64.so
      config/
        config.xml
        savedArmStates.csv
```

Notes:

- `unitree_sdk2` and `cyclonedds` are still expected to be installed on the target machine.
- `z1_ctrl/config/config.xml` must match the real robot network configuration.
- `z1_sdk` and `z1_ctrl` are used only when the controller runs on a non-loopback network.

## 3. Build

If Unitree SDK2 headers are not already in your include path, set one of:

```bash
export UNITREE_SDK2_INCLUDE_DIR=/path/to/unitree_sdk2/include
# or
export UNITREE_SDK2_ROOT=/path/to/unitree_sdk2
```

Build:

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2z1
mkdir -p build
cd build
cmake ..
cmake --build . -j8
```

If `unitree_sdk2` is not in a standard include path:

```bash
export UNITREE_SDK2_INCLUDE_DIR=/path/to/unitree_sdk2/include
```

## 4. Recommended first calibration

Keep the robot safely suspended, then infer the deploy joint map:

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2z1/build
./b2z1_infer_joint_map --network=enp5s0
```

The tool prints:

- `joint_ids_map: [...]`
- `FixStand qs[1]: [...]`

Copy them into:

- `config/policy/velocity/v0/params/deploy.yaml`
- `config/config.yaml`

## 5. Run modes

### Simulation

Loopback mode automatically skips the direct Z1 SDK bridge and falls back to the
full 18-joint `lowcmd/lowstate` path, which is what you want for MuJoCo
validation:

```bash
./b2z1_ctrl --network=lo
```

Expected startup log:

- `Loopback network detected...`
- `Connected to robot.`
- `Loopback network detected, keep Z1 on simulated lowcmd path and skip direct SDK bridge.`

### Real robot

On a real robot network, B2 and Z1 use different transport paths:

- B2 uses Unitree DDS `rt/lowstate` / `rt/lowcmd`
- Z1 uses `unitree_arm_sdk` plus the bundled `z1_ctrl`

Example:

```bash
./b2z1_ctrl --network=enp5s0
```

Controls:

- Handheld controller: `L2 + Up` -> `FixStand`
- Handheld controller: `R2 + A` -> `Velocity`
- Handheld controller: `Y` toggles the arm trajectory state machine
- Handheld controller: `L2 + Down` -> `SlowLieDown`
- Handheld controller: `L2 + B` -> `Passive`

Keyboard fallback:

- `f` -> `FixStand`
- `v` -> `Velocity`
- `p` -> `Passive`
- `w/s`, `a/d`, `q/e` -> base command
- `o` -> toggle arm trajectory
- `l` -> `SlowLieDown`
- `u` -> toggle Z1 diagnostic motion in `Passive` / `FixStand` / `SlowLieDown`

## 6. Arm trajectory

The fixed arm trajectory is configured in:

`deploy/robots/b2z1/config/policy/velocity/v0/params/deploy.yaml`

Relevant fields:

- `commands.arm_pose.start_pose`
- `commands.arm_pose.trajectory.center`
- `commands.arm_pose.trajectory.amplitude`
- `commands.arm_pose.trajectory.period`
- `commands.arm_pose.zero_q`

Current behavior:

- Press `Y` or keyboard `o`
- Arm moves to `start_pose`
- Arm enters the configured fixed trajectory
- Press again to exit trajectory
- Arm returns to `start_pose`
- Arm retracts to `zero_q`

Current real-robot architecture:

- B2 still uses Unitree DDS `rt/lowstate` / `rt/lowcmd`
- Z1 uses a direct `unitree_arm_sdk` low-level channel
- `z1_ctrl` is spawned automatically from `deploy/thirdparty/z1_ctrl/z1_ctrl`
- Z1 command/state no longer depends on ROS2 action or `/joint_states`

The bridge settings live in:

- `deploy/robots/b2z1/config/config.yaml`
  - `z1_sdk.enabled`
  - `z1_sdk.enable_on_loopback`
  - `z1_sdk.control_hz`
  - `z1_sdk.controller_binary_path`
  - `z1_sdk.passive_on_shutdown`

## 7. Z1 diagnostic motion

For safe communication checks before enabling the locomotion policy:

- Put the robot in `FixStand`, `Passive`, or `SlowLieDown`
- Press keyboard `u`
- Only Z1 will execute a small sinusoidal joint motion
- Press `u` again to stop
- Entering `Velocity` automatically disables this diagnostic mode

This is useful to verify:

- Z1 SDK communication is alive
- The commanded joint order is correct
- The sign and direction of the commanded joint are as expected

The diagnostic settings live in:

- `deploy/robots/b2z1/config/config.yaml`
  - `z1_diagnostic.enabled`
  - `z1_diagnostic.toggle_key`
  - `z1_diagnostic.period_sec`
  - `z1_diagnostic.amplitude_q`

## 8. Sim verification

The current repo already has a generic MuJoCo simulator at:

`/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/simulate`

For a quick B2Z1 scene smoke test, build it if needed:

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/simulate
mkdir -p build
cd build
cmake ..
cmake --build . -j8
```

Then point the simulator to the B2Z1 XML from training assets:

```bash
./unitree_mujoco \
  --network lo \
  --robot b2 \
  --scene /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/referent/0506/b2z1_train/src/b2z1/b2z1/xmls/b2z1.xml
```

Notes:

- The simulator uses the non-`g1` bridge path, so `--robot b2` is the expected choice here.
- This verifies scene loading, joystick path, and low-level DDS loopback.

## 9. Deployment terrains in simulation

The deploy simulator can also run with non-flat terrain by changing `simulate/config.yaml`
or overriding the scene path on the command line.

Examples:

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/simulate/build
./unitree_mujoco \
  --network lo \
  --scene src/assets/robots/unitree_b2z1/xmls/scene_b2z1_stairs.xml
```

```bash
./unitree_mujoco \
  --network lo \
  --scene src/assets/robots/unitree_b2z1/xmls/scene_b2z1_slope.xml
```

Available B2Z1 deployment scenes:

- `src/assets/robots/unitree_b2z1/xmls/scene_b2z1.xml`
- `src/assets/robots/unitree_b2z1/xmls/scene_b2z1_stairs.xml`
- `src/assets/robots/unitree_b2z1/xmls/scene_b2z1_slope.xml`
- `src/assets/robots/unitree_b2z1/xmls/scene_b2z1_rough_blocks.xml`
- Policy-in-the-loop validation is done by launching `b2z1_ctrl --network=lo` against the simulator.
- In loopback mode, the real Z1 SDK is intentionally skipped.

## 9. Important notes

- `deploy.yaml` currently carries the previous B2Z1 deploy observation/action layout and arm trajectory state machine.
- The shared deploy observation registry was extended to support:
  - `twist_commands`
  - `arm_pose_command`
  - `foot_contacts_heuristic`
  - `actor_observation_vector`
- If your newly trained policy changed observation ordering or action scaling, update `deploy.yaml` before deployment.
