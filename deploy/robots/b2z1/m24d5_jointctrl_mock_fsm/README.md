# M2.4d5 — MuJoCo-only JointCtrlMockSim FSM integration (B2 + Z1)

**Scope:** Opt-in real `b2z1_ctrl` FSM integration for **MuJoCo loopback only**. Base `State_InteractiveArmIK` observation / ONNX / IK / 20-slot LowCmd code is reused. New alias `JointCtrlMockSim` routes Z1 simulator goals through the existing M2.4d4 `MockOnlyRuntime`, M2.4d1 receiver validation and M2.4c3 safety gate before updating **MuJoCo** Z1 slots. B2 leg actor pathway remains the existing one. Hardware `z1_jointctrl.enabled:true` pre-DDS startup refusal remains. No official Z1 SDK construction, RX hook, `sendRecv`, actual motor modes, or real motor authorization. Synthetic simulator tick evidence is NOT authentic Z1 UDP evidence.

## Verified prerequisites

- Git branch `feature/b2z1-interactive-deploy`, base `8f94aff` or a descendant.
- Official current targeted tracked files clean vs HEAD; installer **refuses** if any targeted file was already changed. Other untracked backup files are not touched.
- The existing M2.3 MuJoCo B2+Z1 scene with **20 actual simulated motor slots**, all of B2Z1 policy, IK, and unitree_sdk2 dependencies operational.

## Install

```bash
cd ~/下载
unzip m24d5_jointctrl_mock_fsm_overlay.zip
cd m24d5_jointctrl_mock_fsm_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh "$REPO"
```

Installer **adds** `deploy/robots/b2z1/m24d5_jointctrl_mock_fsm/`, `config/config_interactive_jointctrl_mock_m24d5_sim.yaml`, and **modifies only** `main.cpp`, `CMakeLists.txt`, `include/State_InteractiveArmIK.h`, and `src/State_InteractiveArmIK.cpp`. Does not modify default `config/config.yaml` or existing shadow config. Does not delete or stage files. Backup/rollback via Git is possible because installer verifies original target files were clean.

## Full controller build — required on developer workstation, not performed here

```bash
cd "$REPO/deploy/robots/b2z1"
cmake -S . -B build
cmake --build build --target b2z1_ctrl -j2
```

If compilation fails, do not continue to simulator test; send the **first** compiler diagnostic plus `git diff --stat` and `git diff --check`.

## Simulation-only startup — AFTER build passes

1. Start the exact known-working **M2.3 B2+Z1 MuJoCo scene**, with DDS network `lo`. Keep all real robot network interfaces detached/unbridged from the simulator.
2. In a separate terminal:

```bash
cd "$REPO/deploy/robots/b2z1/build"
export B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE
./b2z1_ctrl --network=lo --config=config/config_interactive_jointctrl_mock_m24d5_sim.yaml
```

3. From **MuJoCo** FixStand, `RT + B` enters **JointCtrlMockSim** (replaces the previous RT+B shadow entry only in the new config). `i/k`, `j/h`, `u/n` are the existing ±1cm constrained Z1 IK XYZ keys; `w` starts the existing bounded forward-only B2 motion after warmup, `SPACE` stops walking/freezes arm; `LT+B` returns Passive. **Do not use this config against actual robotics hardware**, even in a lab connected to a robot.

4. Check that after >=3 advanced SIM feedback ticks the Z1 motor targets come from bounded MockOnlyRuntime commands rather than directly from IK. The actor uses the same 799D observation including measured B2+Z1 simulated joint values.

**Strict limitations:** synthetic q/dq/state/temp used to drive M2.4d1 are **fixtures**; the only real feedback consumed by this path is MuJoCo's 20-slot LowState. No actual UDP peer, decoder, motor temperature, independent emergency stop, or physical fault guarantees are present. A cooperative `flock` at `/tmp/b2z1_m24d5_mock_<uid>.lock` prevents only cooperating processes. The mock sink records up to 10,000 control ticks (~50s at 200Hz) and deliberately faults thereafter. On any feedback/safety failure, the existing SIM state latches fault and stops publishing further motion; it is **not** a physical safe stop or E-stop.

## Tests and progress gates

- `bash run_offline_tests.sh "$REPO"` compiles/runs M2.4d5 + the **actual** M2.4d4/d1/c3/b2 source files (only simulation C++), checks installer fixture and static isolation.
- `cmake --build ... --target b2z1_ctrl` tests full firmware-independent C++ integration on your PC.
- MuJoCo manual joint & walking validation is the *remaining* acceptance test.
- No real Z1 JOINTCTRL path, hardware tests, or runtime SDK RX authorization are unlocked.

### Regression checks

```bash
cd "$REPO"
git diff --check
git diff --stat
git diff -- deploy/robots/b2z1/main.cpp deploy/robots/b2z1/CMakeLists.txt \
  deploy/robots/b2z1/include/State_InteractiveArmIK.h \
  deploy/robots/b2z1/src/State_InteractiveArmIK.cpp
```

Stop if any existing `M2.3`, `JointCtrlShadow`, `InteractiveArmIK` behavior changes when running their original configs.
