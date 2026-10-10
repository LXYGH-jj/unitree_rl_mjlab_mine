# B2+Z1 键盘末端控制（Interactive RL + MuJoCo IK）

## 适用范围

* 基于 `LXYGH-jj/unitree_rl_mjlab_mine` 仓库 `go2_deploy` / `local-modified` 分支中的 `src/assets/robots/unitree_b2z1/xmls` 模型。
* 复用已验证的 `b2z1_low_policy_jit.pt`（799 输入、18 输出）以及 `lowlevel_core.py` / `config_lowlevel.yaml`。
* B2 12腿：Interactive RL 位置目标 + PD；Z1 6关节：MuJoCo Jacobian 阻尼最小二乘位置 IK + PD。
* **仅仿真**，不可直接用于控制真实B2、Z1。
* 未修改 `b2z1_ros2_ws`，未修改已有模型/XML/原有脚本。

## 文件位置建议

解压后把目录放到：

```
/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/tools/b2z1_interactive_keyboard/
```

如果你的仓库在其他目录，请改下面的 `--xml` 参数。

## 环境

```
source /home/vi/miniconda3/etc/profile.d/conda.sh
conda activate b2z1_wbc
python -c 'import mujoco, torch, yaml; print("mujoco", mujoco.__version__, "torch", torch.__version__)'
```

要使用Python MuJoCo Viewer。如果该环境尚未安装 MuJoCo Python 接口，可在确认环境隔离后安装相应版本（例如 `python -m pip install 'mujoco==3.3.6'`）；若已存在，勿重复安装/升级。

## 启动

在 `unitree_rl_mjlab` 工程目录下运行：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab

WBC_DIR=/home/vi/lxy/b2z1_ws/B2Z1_Interactive_MuJoCo_Sim2Sim/b2z1_interactive_sim2sim

conda deactivate 2>/dev/null || true
source /home/vi/miniconda3/etc/profile.d/conda.sh
conda activate b2z1_wbc

# 请不要同时运行原来的 sim2sim_interactive_stage1.py 或 C++ unitree_mujoco；
# 该脚本会在自己的MuJoCo进程里加载相同的XML和模型。

python tools/b2z1_interactive_keyboard/b2z1_keyboard_ik.py \
  --xml src/assets/robots/unitree_b2z1/xmls/scene_b2z1_stage1_official_b2_inertia.xml \
  --wbc-dir "$WBC_DIR" \
  --policy "$WBC_DIR/b2z1_low_policy_jit.pt" \
  --arm-kp 60 --arm-kd 4 \
  --csv /tmp/b2z1_keyboard_ik.csv
```

若上面 `scene_b2z1_stage1_official_b2_inertia.xml` 尚未生成，可先改用：

```
--xml src/assets/robots/unitree_b2z1/xmls/scene_b2z1.xml
```

但是建议先保留你前面通过测试的官方惯量实验场景，不要在比较过程中混用场景。

## 键盘按键（先点击MuJoCo Viewer窗口）

|按键|效果|
|---|---|
|W/S|B2前进/后退速度指令每次±0.12 m/s|
|A/D|B2左/右横移速度指令每次±0.06 m/s|
|Q/E|B2正/负转向速度指令每次±0.12 rad/s|
|空格|机身速度指令归零（**不是**实机急停）|
|I/K|末端目标前/后移动，每次±1.5 cm|
|J/L|末端目标左/右移动，每次±1.5 cm|
|U/O|末端目标上/下移动，每次±1.5 cm|
|M|切换末端目标参考系：`body`（随机器人走）/`world`（固定世界位置）|
|T|将末端目标重置为当前位置，便于从不可达位置恢复|
|P|暂停/恢复仿真|
|关闭Viewer窗口|退出程序并可保存CSV|

橙色小球为目标末端位置；Z1真实 `tcp` 刚体应尝试跟踪橙色球。

## 建议先验证的三个步骤

1. 默认 `cmd=0`，按 `I/K/J/L/U/O`，看Z1末端是否朝键盘指定方向移动；保持B2稳定。
2. 按 `W` 让B2小速度走，保持 `body` 模式，再控制Z1末端；观察腿部姿态与机身稳定性。
3. 按 `M` 切换 `world` 模式，**先把机身速度恢复零**，小幅改变末端目标。之后再尝试短距离行走；由于世界固定目标可能很快超出机械臂工作空间，不宜长时间运动。

## 技术边界/下一步

* 本程序使用**位置IK**（3×6 TCP位置雅可比），不控制末端姿态、接触力，也不是完整全身力优化。
* B2策略使用Z1实际关节位置/速度和共享的TCP目标来构建799维观测，因此可以有腿部姿态的反馈响应。是否真的出现可观测的稳定性补偿，需通过实验比较验证，不应只凭视觉判断。
* `arm-kp` / `arm-kd` 是仿真演示参数（默认为60/4），与Gym静态对照用的5/0.5不同。运行结果不能直接用于与之前的静态机械臂Sim-to-Sim速度表严格对比。
* 运行时默认使用训练阶段的名义 `arm_base_offset=[0.20,0,0.09]` 来构建末端观测，模型实际连杆 `link00` 位置参与IK。实际装配阶段要重新标定两者的一致性。
* Python MuJoCo脚本直接调用模型，不经过SDK2 LowCmd/LowState。此阶段检验“协同控制算法与键盘交互”，不是部署接口验证。后续再迁入仓库C++ B2Z1 FSM/SDK2系统和最终的 `b2z1_ros2_ws`。
* 如果窗口卡死或机器人倒地，关闭Viewer，重新启动；不要试图把此文件用于实机控制。
