# M2.4c2 — JOINTCTRL FSM wiring, MuJoCo shadow-only

Base Git commit: `7aa7c08`, branch `feature/b2z1-interactive-deploy`.

This step registers `State_JointCtrlShadow` in the actual `b2z1_ctrl` FSM.
It reuses real Interactive `799D` observation/ONNX/feedback adapter, computes
independent simulated EE/IK joint goal previews, logs the 12 B2 policy outputs
and preview Z1 six joints, but **does not apply these previews to any motor**.
The simulated robot holds its measured entry joint positions; ONLY `lo` MuJoCo
DDS is permitted. It cannot exercise vendor JOINTCTRL or qualify any hardware.

It intentionally leaves `State_InteractiveArmIK` and all M2.3 YAML untouched.
`State_JointCtrlShadow` is new and has separate YAML and ID 7.
The M2.4c1 `ShadowBridge::onZ1(..., true)` is used ONLY for confirmed-new
**synthetic MuJoCo tick**, not actual UDP packet freshness. NEVER transplant
this value into real Z1 SDK logic.

## Developer PC

```bash
cd ~/下载
unzip m24c2_jointctrl_fsm_overlay.zip
cd m24c2_jointctrl_fsm_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check > /tmp/m24c2.diff
less /tmp/m24c2.diff
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh "$REPO"
cmake -S "$REPO/deploy/robots/b2z1" -B "$REPO/deploy/robots/b2z1/build"
cmake --build "$REPO/deploy/robots/b2z1/build" --target b2z1_ctrl -j2
git -C "$REPO" diff --check
```

Do NOT attempt physical hardware, even if C++ builds.
Check whether stage `State_JointCtrlShadow` logs on loopback. For sim use the
`config_interactive_jointctrl_shadow_m24c2_sim.yaml` (new file) with
`B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE`. The sim scene must expose 20
motor slots; the arm in this shadow state is held instead of moved. Existing
M2.3 configuration and command remain the basis for full motion validation.

The new sim-only FSM transition from FixStand is `RT + B.on_pressed`.
In preview: keyboard i/k/j/h/u/n changes IK goal ±1cm; output is logged but
NOT sent to any actuator. Return to FixStand with `LT + Up`.

### Git workflow AFTER offline tests and full build

```bash
cd "$REPO"
git status --short
git add -- \
  deploy/robots/b2z1/main.cpp \
  deploy/robots/b2z1/CMakeLists.txt \
  deploy/robots/b2z1/include/State_JointCtrlShadow.h \
  deploy/robots/b2z1/src/State_JointCtrlShadow.cpp \
  deploy/robots/b2z1/config/config_interactive_jointctrl_shadow_m24c2_sim.yaml \
  deploy/robots/b2z1/m24c2_jointctrl_fsm_overlay

git diff --cached --check
git --no-pager diff --cached --stat
git --no-pager diff --cached --name-only
```

Do not run `git add -A`, `git clean`, or hard reset. Commit/push only after
successful compile and simulation inspection. Clone into a **new** directory
on the physical deployment PC and run offline checks before connecting robots.

## Not yet done

1. Vendor SDK JOINTCTRL live feedback verified with independent new UDP receive
   packet evidence (or another specifically validated freshness method).
2. Joint position interface with real actuator authorization, fail-safe controlled
   stop and e-stop.
3. Fully coupled B2 SDK2 and real Z1 SDK control/observation.
4. Electrical root cause and fuse/power inspection.

**Current M2.4c2 is SIM ONLY.** It is NOT a complete real deployment.
