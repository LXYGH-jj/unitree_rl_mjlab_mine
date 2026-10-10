# Stage 1 torque/actuator trace, independent diagnostic

This is a trace-only fork of the two static-arm scripts previously tested by the user. It does **not** overwrite those scripts, change gains, alter trained weights, or address hardware. It records actor action, joint position/velocity, pre-PD target, and final-physics-substep applied clipped torque for twelve leg joints. The new CSV files have suffix `_torque_trace.csv`.

Compare the **distributions** of actions/torques over the same steady-state time window, not each timestamp numerically: physical trajectories between Gym and MuJoCo diverge. The one-step output from Gym is `env.torques` (author calculated torque) while MuJoCo output is `data.ctrl` (actuator command). Both use torque clipping. `torque_sat_last` is the number of 12 leg joints at or past the torque limit on the final physics substep. MuJoCo additionally reports `torque_sat_substeps`, the sum over all four physics substeps. `torque_leg_peak` is a final-physics-step reading for Gym and maximum over physics substeps for MuJoCo; do not interpret these as exactly identical metrics.

## MuJoCo
```bash
cd /home/vi/lxy/b2z1_ws/B2Z1_Interactive_MuJoCo_Sim2Sim/b2z1_interactive_sim2sim
python -m py_compile stage1_mujoco_torque_trace.py stage1_gym_torque_trace.py

env LD_LIBRARY_PATH="$CONDA_PREFIX/lib" TORCH_EXTENSIONS_DIR="$CONDA_PREFIX/torch_extensions" MAX_JOBS=2 PYTHONNOUSERSITE=1 \
python stage1_mujoco_torque_trace.py \
  --xml /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/src/assets/robots/unitree_b2z1/xmls/scene_b2z1_stage1_official_b2_inertia.xml \
  --policy ./b2z1_low_policy_jit.pt --vx 0.30 --seconds 6 --warmup 1.0 \
  --csv ./stage1_official_b2_torque.csv | tee stage1_official_b2_torque.log
```

## Isaac Gym
```bash
env LD_LIBRARY_PATH="$CONDA_PREFIX/lib" TORCH_EXTENSIONS_DIR="$CONDA_PREFIX/torch_extensions" MAX_JOBS=2 PYTHONNOUSERSITE=1 \
python stage1_gym_torque_trace.py \
  --interactive-root /home/vi/lxy/b2z1_ws/Interactive-Navigation-for-legged-manipulator/low_level_WBC \
  --policy ./b2z1_low_policy_jit.pt --vx 0.30 --seconds 6 --warmup 1.0 \
  --csv ./stage1_gym_torque.csv | tee stage1_gym_torque.log
```

Ensure both tests are on the same fixed-arm protocol, and check for any exception. No Isaac Gym runtime validation has been performed by the assistant.
