# Interactive B2Z1 → MuJoCo Sim-to-Sim（第一阶段）

**定位：独立的 MuJoCo Python 验证程序；不会连接真实机器人，也不使用 DDS/ROS。**

使用 ALORE 中提取的 `lowlevel_core.py` 实现 71维当前观测 + 18维特权占位 + 10×71 历史（共799维），加载 Interactive 原低层 Actor 的 TorchScript 推理文件，输出18维动作，取前12维控制 B2；Z1保持Interactive默认关节角度。它的目标是验证Actor+观测+MuJoCo物理的一致性，不等于已经实现腿臂末端IK控制。

## 0. 环境

原训练 PC `b2z1_wbc` Python 3.8 + torch 1.10 (已复现预训练权重)。MuJoCo Python 3.1.6 提供 Python3.8 轮子：

```
conda activate b2z1_wbc
python -m pip install 'mujoco==3.1.6'
```

请使用已经成功加载原始 `model_57000.pt` 的同一环境进行导出与验证。不需要安装完整ALORE。

## 1. 在训练电脑导出TorchScript

```
cd ~/lxy/b2z1_ws/Interactive-Navigation-for-legged-manipulator
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
env LD_LIBRARY_PATH="$CONDA_PREFIX/lib" TORCH_EXTENSIONS_DIR="$CONDA_PREFIX/torch_extensions" MAX_JOBS=2 PYTHONNOUSERSITE=1 \
python /path/to/export_jit_on_training_pc.py --output "$HOME/lxy/b2z1_low_policy_jit.pt"
```

`export_jit_on_training_pc.py` 使用模型权重中的Actor，并显式固定 `hist_encoding=True`。输出shape：[1,799]→[1,18]。本脚本未在用户主机上实测，请先检查终端中Parity diff记录及EXPORTED。

## 2. MuJoCo模型

请检出 / 保持用户的 `LXYGH-jj/unitree_rl_mjlab_mine` 的 `go2_deploy` 分支。XML在仓库根目录的：

```
src/assets/robots/unitree_b2z1/xmls/scene_b2z1.xml
```

所需 `b2z1.xml` 和 `assets/` 相对目录均在同一XML目录下。脚本通过MuJoCo直接加载该场景，**不需要 `unitree_mujoco` 和 `b2z1_ctrl` 同时启动**。

## 3. 执行第一次 Sim-to-Sim

```
REPO=/path/to/unitree_rl_mjlab_mine
cd /path/to/b2z1_interactive_sim2sim
python sim2sim_mujoco.py \
  --xml "$REPO/src/assets/robots/unitree_b2z1/xmls/scene_b2z1.xml" \
  --policy "$HOME/lxy/b2z1_low_policy_jit.pt" \
  --vx 0.2 --seconds 10 --warmup 1.0 --realtime
```

若无图形显示或想快速看控制日志：末尾改为 `--headless`。

可选参数：`--vx 0.4`、`--wz 0.15`、`--physics-dt 0.002`。第一阶段先不要测试复杂地形。

## 4. 与已有C++ DDS仿真链路的关系

当前Python脚本只用于验证模型与控制语义是否可复现；它不会调用B2 SDK2、Z1 SDK、ROS或高层策略。通过后将同样的观测构造移植到 `deploy/include/isaaclab/envs/mdp/observations/observations.h`，并导出一个静态输入shape为[1,799]、输入名为`obs`的ONNX，配置对应 `deploy.yaml`。此时才能通过两终端 `unitree_mujoco --network lo --robot b2 --scene ...` + `b2z1_ctrl --network=lo` 形成真正的 DDS 在环Sim-to-Sim。

现有 `deploy.yaml` 采用 Isaac Lab 格式，旧Actor的观测布局与Interactive并不一致；不能直接用新的ONNX覆盖旧 `policy.onnx`。

## 5. 一致性注意事项

- Interactive策略腿部顺序是 `FR FL RR RL`，脚本明确按该顺序从MuJoCo读取关节，禁止将模型原始的 `FL FR RL RR` 直接交给策略。
- 保留两份数据含义：当前71维观测和**之前**10帧历史；不是最新观测简单重复10次。
- 策略低层控制频率 50Hz，本脚本默认MuJoCo物理200Hz；模拟电机为torque motor，使用B2训练仿真Kp=360、Kd=5。**不是实机参数。**
- Z1固定在训练默认姿态，不通过Actor最后6维控制；末端输入 `ee_local_xyz` 是该静态姿态在模型中的末端相对位置**近似**。这不同于Interactive对全身末端目标进行在线追踪，待二阶段IK校准。
- 该脚本对关节动作进行了ALORE部署限幅；相比Isaac Gym原策略的力矩饱和，控制结果可能有差异。初版旨在发现模型/观测/动力学问题，不保证数值完全复现。
- 由于双方模型惯性/接触参数可能有差异，成功只能证明MuJoCo仿真里运行正常，不能据此直接实机部署。
- 本文件已做Python语法检查。当前执行环境没有安装MuJoCo 3.1.6，也没有真实模型/权重的运行副本，因此**尚未进行完整运行测试**。
