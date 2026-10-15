# M2.4e2h — Z1 SDK LOWCMD bench-preparation correction

## Prerequisite already confirmed

The repaired Z1 successfully operates with the vendor's `z1_ctrl k` keyboard
controller. This establishes a hardware baseline; it **does not** qualify our
LOWCMD Worker or its controller stop behavior.

## Changes

1. The SDK startup command and all SDK control-cycle commands now use
   `arm._ctrlComp->armModel->inverseDynamics(q, qd, Zero, Zero)`
   for gravity/dynamics feedforward, as in the vendor's `examples/lowcmd_development.cpp`.
   This is not a torque-controller guarantee and depends on correct Z1 model,
   mounting orientation and actual gripper/load.
2. The SDK backend now requires `--has-gripper yes|no` explicitly, rather than
   silently constructing the no-gripper SDK on a gripper-equipped Z1. Prior
   vendor console logs indicated `The arm has gripper`, but confirm the actual
   configuration independently before any SDK test.
3. A purely numerical **50 Nm fail-closed sanity bound** rejects absurd or
   nonfinite feedforward commands. It is **NOT** a validated torque limit for
   any Z1 motor; real rated per-joint torque and gains still need verification.
4. The startup order / FSM transitions / socket and planner behavior otherwise
   remain those of E2d. This patch does **NOT** resolve proven UDP RX freshness,
   safe stopping on SIGTERM or socket disconnect, actual torque limits, or
   500 Hz hard real-time schedule. Thus SDK operation is STILL NOT QUALIFIED.

## Install and test — developer PC, no robot

```bash
cd ~/下载
unzip m24e2h_z1_bench_ff_patch.zip
cd m24e2h_z1_bench_ff_patch
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2h_z1_bench_ff_patch/run_offline_tests.sh" "$REPO"
bash "$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/run_sdk_linkcheck.sh" "$REPO"
```

`run_sdk_linkcheck.sh` compiles and links but NEVER executes the real SDK path.
Do not run the SDK backend at this stage and do not re-run vendor `z1_ctrl k`
concurrently with the self-developed Worker.

## Existing state of main FSM

E2g already builds and remains OFF for B2 physical output and LOCKED for Z1
SDK target sending. Do not change those gates as part of this patch.

## Remaining before actual bench LOWCMD test

Confirm vendor `config.xml`, gripper flag, level orientation, support/physical
emergency stop, model mass/load, `LOWCMD` PD gains and rated motor limits.
Obtain observed joint sample and a definite supported stop/handoff procedure.
Bench movement can only be enabled after those are checked, with an attendant.
