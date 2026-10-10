# ALORE 低层 B2+Z1 WBC 独立化：源码核查与第一版提取代码

> 源码基准：`Zhihaibi/ALORE_Legged_Manipulator`, branch `main`, commit `870e86d7976eb74b09507ffc12f237d4b09b869f`。Interactive 对照基准：`Zhihaibi/Interactive-Navigation-for-legged-manipulator`, commit `8c5ea5ce2e1e97392e1bf3c0302a73b8b4e26c12`。
>
> **本包是已可离线单元测试的“低层算法内核 + 模型导出程序 + 实机只读探针”，不是已完成实机电机闭环验收的控制器。所有附带代码默认都不会发送电机指令。**

## 1. 对原始 `deploy_real_b2z1_obj.py` 的结构审计

| 源码部分（原始行号） | 功能 | 低层独立版本处置 |
|---|---|---|
| 1–28 | PyTorch, Unitree SDK2, ROS1, 工具函数 | 将策略核心与机器人 I/O 拆开 |
| 29–118 | 低层策略载入、71维观测状态、DDS输入输出、运动模式切换 | 保留低层数据语义；拆分 DDS I/O，不在推理内核中创建Publisher/MotionSwitcher |
| 119–160 | 高层策略及物体相关变量 | **删除** |
| 161–177 | ROS1 Z1反馈、末端信息、任务订阅、等待初始化 | 只保留 `arm_current_state` 读取；末端目标改成明确输入 |
| 179–224 | 物体、视觉定位、规划回调 | **删除** |
| 226–251 | Z1状态、末端状态、B2 LowState 回调 | 把原始数据采样适配到 `RobotSample` |
| 253–317 | DDS发送、站立、等待Start/A按键 | 算法内核不发送；这些属于**后续单独验收的安全状态机/执行层** |
| 319–345 | 步态相位和4路正弦时钟 | 在内核中保留 |
| 347–465 | 关节状态→71/799观测→TorchScript→18维动作→12维腿部目标 | **完整提取主要计算逻辑**，明确做数据校验 |
| 468–583 | 高层策略、物体速度、物体操作动作 | **删除** |
| 586–700 | 主循环：高层调用+任务状态处理+低层调用 | 改为单独测试入口；不再运行 `high_level_controller()` |

原始源码：[B2 `deploy_real_b2z1_obj.py`](https://github.com/Zhihaibi/ALORE_Legged_Manipulator/blob/main/Deployment/B2_deploy/deploy_real_b2z1_obj.py)；[Z1 `z1_control.py`](https://github.com/Zhihaibi/ALORE_Legged_Manipulator/blob/main/Deployment/Z1_deploy/z1_control.py)。

### 2. 低层WBC所需输入、输出

**输入**：
- B2：12个关节位置/速度 + IMU(w,x,y,z四元数和角速度)。
- Z1：`arm_current_state` 中6个机械臂关节位置和6个关节速度。
- 期望底盘速度：`[vx,vy,wz]`，目前无需感知、导航、物体识别。
- **期望机械臂末端目标的相对坐标**：3维（必须确认定义，不等于末端实际测量位置）。
- 历史10帧71维本体观测；上次12维腿部策略动作；步态相位和4路时钟。

**输出**：18维动作，其中当前首要任务只使用前12维腿部动作生成位置目标。Z1关节目标由独立 Z1 控制程序根据人工指令/预设轨迹生成，而非直接使用低层Actor后6维。

观测对照（0起始）：

| 位置 | 长度 | 意义 |
|---|---:|---|
| 0:2 | 2 | roll/pitch |
| 2:5 | 3 | 机身角速度×0.25 |
| 5:23 | 18 | 关节位置减默认值×1.0 |
| 23:41 | 18 | 关节速度×0.05 |
| 41:53 | 12 | 前一次腿部动作 |
| 53:57 | 4 | 接触观测为零 |
| 57:60 | 3 | 速度命令×[2,2,0.25] |
| 60:63 | 3 | 机械臂末端相对**目标**位置，需要语义校核 |
| 63:66 | 3 | 姿态目标占位零 |
| 66:67 | 1 | gait_phase |
| 67:71 | 4 | 正弦时钟 |
| 71:89 | 18 | 特权占位；ALORE的索引5（摩擦）为1，其余为0；hist模式不进入Actor |
| 89:799 | 710 | **前10帧**的71维观测，FIFO |

### 3. 五个必须处理的语义差异/风险

1. **末端目标与测量位置**：原始ALORE `sub_ee_state_callback` 从 `/hand_current_state` 的 `msg.data[:3]` 抽取**真实当前末端位置**，塞到 `obs_proprio[60:63]`。Interactive训练端 `compute_observations()` 用 `quat_rotate_inverse(base_quat, goal_world - arm_base_world)` 的**期望末端位置**。这是不同的物理量，且训练端包括基座旋转和机械臂安装偏移。低层内核刻意要求调用者提供显式 `ee_local_xyz`，绝不默默替换。
2. **默认角度**：ALORE仓库配置把B2腿的髋角写成近零，而Interactive训练模型里左腿+0.1、右腿-0.1。当前提取版配置设置为**按策略FR/FL/RR/RL重排的训练默认角度**，仅用于对齐验证；在实机启用前必须核实与硬件零位、站姿、力控模式和机械臂负载兼容。
3. **关节顺序**：Interactive URDF加载顺序 `FL,FR,RL,RR,arm`，而重排策略输入/输出为 `FR,FL,RR,RL,arm`。ALORE实机B2电机列表是 `[0..11]`，不可未经核实就当作相同顺序。`lowlevel_core.py` 要求调用者传入**策略顺序**，不处理硬件重映射。
4. **历史队列时序**：原始ALORE先把10帧旧历史拼到当前观测，再把当前帧压入历史；第一帧历史全零。这个顺序已在提取版中保留，与Interactive `compute_observations()` 的历史更新时序相容。
5. **安全性**：原版 `low_level_controller` 把Actor裁剪到±100，并用作者实机 `Kp=360,Kd=5`、站立 `Kp=700,Kd=10`直接驱动B2。±100只是训练clip，不是**运动安全限幅**，而这些增益也**不能直接认定适用于你的实机**。原始B2主循环没有完整的反馈过期、推理耗时、急停冗余和切换保护。Z1独立循环约25Hz目标更新、2ms内部发送，与B2的50Hz是异步的，需要时间戳保护。

### 4. 已完成的提取模块

```text
lowlevel_core.py              纯算法内核：不依赖Isaac Gym、ROS、SDK2
config_lowlevel.yaml          仅保留低层策略参数，显式不包含high_level
export_jit_on_training_pc.py  在已验证训练电脑导出TorchScript并校验
 offline_check.py              799维输入、18维输出的完全离线测试
b2_readonly_probe_ros1.py     B2 DDS + Z1 ROS1 状态读取与推理，只读、不发电机命令
tests/test_core.py            数据维度、时序、启动保持、状态合法性测试
```

这里特意保留当前`model_57000.pt`导出的历史Actor输入 `[1,799]`、输出 `[1,18]`，没有把它误作高层的9维策略。`model_57000.pt` 本身是checkpoint，不能直接 `torch.jit.load()`。

### 5. 训练电脑：导出一次低层模型

无需安装ALORE。在你已能运行 `b2z1_preview.py` 的训练电脑：

```bash
conda activate b2z1_wbc
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH
export PYTHONNOUSERSITE=1
cd /home/vi/lxy/b2z1_ws/Interactive-Navigation-for-legged-manipulator
# 将 export_jit_on_training_pc.py 复制到任意可读取位置，例如当前项目目录

env LD_LIBRARY_PATH="$CONDA_PREFIX/lib" \
TORCH_EXTENSIONS_DIR="$CONDA_PREFIX/torch_extensions" \
MAX_JOBS=2 PYTHONNOUSERSITE=1 \
python /path/to/export_jit_on_training_pc.py \
  --output /home/vi/lxy/b2z1_low_policy_jit.pt
```

必须看到5组随机输入的最大差异都小于`1e-5`且文件成功输出，才复制到部署电脑。**本脚本已做静态编译检查，但由于当前执行环境没有你的checkpoint/GPU，未能进行真实权重导出验证。**

### 6. 训练电脑或部署电脑：脱离仿真的模型干运行

无需安装ALORE全仓库。在已有兼容PyTorch、NumPy、PyYAML的Python环境执行：

```bash
cd /path/to/alore_wbc_lowlevel_only
python -m unittest discover -s tests -v
python offline_check.py \
  --model /path/to/b2z1_low_policy_jit.pt --steps 25
```

本地单元测试通过，只代表数据拼接及纯算法路径通过。测试输入为合成状态，**不能用于判断实机可否安全行走**。

### 7. 部署电脑：读取真实B2和Z1、推理但**不驱动**

如果部署电脑具备 ALORE 的 `unitree_sdk2py`、ROS1的 `rospy`、`std_msgs`、并且 Z1 原版节点 `z1_control.py` 已在安全条件下发布 `/arm_current_state`：

```bash
# network根据部署电脑真实网卡改，ee-xyz仅作观测测试样例，
# 不能认为[0.35,0,0.35]就是已对齐的真实末端目标。
python b2_readonly_probe_ros1.py \
  --network <B2_网卡名> \
  --model ./b2z1_low_policy_jit.pt \
  --ee-xyz 0.35 0.0 0.35 --duration 12
```

这个程序只创建B2 `/lowstate` DDS订阅和Z1 ROS1状态订阅，打印观测/动作/目标。**不会创建B2 `LowCmd` 发布器，不会发Z1动作，也不会执行运动控制权切换。**

如果你的部署电脑只有 ROS2 Humble 而没有 ROS1：无需为了训练电脑安装ROS1；下一步应把该只读探针的 Z1 数据源替换为你已工作的ROS2或Z1 SDK状态接口（只改 I/O，不改 `lowlevel_core.py`）。

### 8. 从只读到实机真正“动起来”仍需哪些验收？

当前交付不包含电机使能/闭环控制功能。必须先完成：

1. 在**同一份录制的B2/Z1状态序列**上，对比 Interactive/ALORE 与提取版的71/799维观测、Actor动作（逐元素对比）。
2. 在MuJoCo中将 `lowlevel_core.py` 接到用户现有 `unitree_rl_mjlab_mine/go2_deploy` 仿真低层消息流，完成长时间姿态、速度、运动测试。
3. 核实真实B2腿部SDK电机ID/零位/控制模式、Z1关节原点、目标末端坐标系；修正末端目标问题。
4. 部署电脑上完成状态超时、关节限幅、动作速率限制、离线/在线急停、独占低层控制权与执行频率验证；确认实际硬件合理Kp/Kd，不沿用高增益默认值。
5. 可靠悬挂/支撑条件下逐步测试；再在隔离测试区域进行低速地面闭环。

### 9. 开源仓库与许可

- [ALORE](https://github.com/Zhihaibi/ALORE_Legged_Manipulator)（源码含独立B2、Z1部署实现）。
- [Interactive](https://github.com/Zhihaibi/Interactive-Navigation-for-legged-manipulator)（提供低层训练/仿真及checkpoint）。
- 本包为对代码结构和算法语义的独立提取/重写示例，不拷贝宇树SDK动态库、训练权重或第三方二进制。直接复用或分发原项目源码时应遵守其许可协议。
