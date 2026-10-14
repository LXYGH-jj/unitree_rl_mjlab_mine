# M2.4c1 — B2+Z1 JOINTCTRL OFFLINE integration + Git handoff

Target: `LXYGH-jj/unitree_rl_mjlab_mine`, branch `feature/b2z1-interactive-deploy`.

## Critical scope

This package is **NOT** the real-hardware JOINTCTRL FSM and **NOT** a complete
MuJoCo regression. It composes the repository's real `adaptFeedback`,
`ObservationBuilder` and `InteractivePipeline` with a mock-injected policy and
software-only 12-leg / 6-arm separation. It has **no SDK, DDS, FSM entry or
motor publishing**. Installing it cannot activate Z1 or B2. Future integration
will add a separately audited live vendor SDK receive path and a hard-locked
real-hardware FSM after physical electrical inspection.

Why this stage: M2.3 loopback sim proves basic B2 + IK behavior, but Z1 JOINTCTRL
hardware was not in that loop. The GitHub branch lacks locally untracked
`sim_closed_loop/joint_target_guard.cpp` and M2.3 config, and the referenced ONNX
model is under ignored `build/`. A clean second-PC clone therefore cannot be
assumed reproducible.

## Developer-PC steps

```bash
cd ~/下载
unzip m24c1_jointctrl_shadow_git_handoff.zip
cd m24c1_jointctrl_shadow_git_handoff/m24c_shadow_integration
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
bash run_offline_tests.sh "$REPO"          # real 799D pipeline, mocked actor
bash install.sh "$REPO"                    # copy-only install
python3 "$REPO/deploy/interactive_b2z1/m24c_shadow_integration/tools/prepare_transfer.py" "$REPO"
python3 "$REPO/deploy/interactive_b2z1/m24c_shadow_integration/tools/prepare_transfer.py" "$REPO" --prepare-model
# After checking model hash and onnx size:
python3 "$REPO/deploy/interactive_b2z1/m24c_shadow_integration/tools/prepare_transfer.py" "$REPO" --stage
cd "$REPO"
git diff --cached --check
git diff --cached --stat
git diff --cached --name-only
# Audit all staged changes carefully, especially tracked M2.4b1 LOWCMD sources.
git commit -m "Preserve B2Z1 M2.3 baseline and add M2.4c JOINTCTRL offline integration"
git push -u origin feature/b2z1-interactive-deploy
```

**Do not** use `git add -A`, `git clean -fd`, `git reset --hard`, or commit local
backup directories. Do not commit generated build outputs, crash logs or
compilation objects. The `--stage` manifest selectively includes source files
from the previously installed M2.4b2/M2.4b3 and shadow files, plus the
M2.3 model/config and existing CMake source dependency.

`--prepare-model` copies the M2.3 ONNX to the versionable `models/` location
and creates `config_interactive_arm_ik_m23_portable_sim.yaml` without modifying
the original config. The model SHA256 is printed: record it and compare on
machine B. GitHub's regular file size cap requires LFS for very large files.

## On the other computer (NO hardware startup)

```bash
git clone -b feature/b2z1-interactive-deploy \
  git@github.com:LXYGH-jj/unitree_rl_mjlab_mine.git
cd unitree_rl_mjlab_mine
bash deploy/interactive_b2z1/m24c_shadow_integration/verify_clone.sh "$PWD"
sha256sum deploy/interactive_b2z1/models/b2z1_interactive_policy.onnx
git status --short
# After configuring this PC's SDK/CMake dependencies, perform a FRESH build.
# Do not run b2z1_ctrl on a real robot at this stage.
```

## Do NOT assume hardware safety

1. `z1_sdk_controller.cpp` remains the OLD **LOWCMD** class, guarded by M2.4b1.
   The real JOINTCTRL runtime module is not installed/active yet.
2. `deploy/robots/b2z1/main.cpp` still contains real B2 ReleaseMode startup
   and may construct that old Z1 controller depending on config. **Do not start**
   it on a robot. No real network is involved in these offline tests.
3. The SDK feedback getter is not guaranteed to prove a newly received UDP packet.
4. Vendor JOINTCTRL and MuJoCo B2+Z1 must still have separate read-only,
   mode-switch and actuator tests; no real motion until fuse incident root-cause
   investigation and inspection is complete.
5. Actual mount ≈ `[0.03,0,0.08]m` for IK; ONNX nominal training mount is
   `[0.20,0,0.09]m`. Keep this transform as in existing adapter.

## Offline preview scope

`Mode::Stand` accepts a zero base velocity with up to 10 cm EE goal displacement
for **offline experiment only**; `Mode::Walk` accepts vx ≤0.12 m/s and ≤3 cm EE
displacement. These are not validated real-world working envelopes.  The IK
joint goal is marked `unqualified`: the existing safety core/physical limits
must approve it later. B2 normalized action outputs are **not** motor targets.
