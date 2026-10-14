# M1.5 — Interactive B2+Z1 SDK2 MuJoCo 闭环（隔离测试）

**当前实现限制：仅 x86_64 + MuJoCo，本程序绝不能接到真实 B2/Z1。**

与已完成的 `shadow_sim` 不同，这个测试程序包含 SDK2 LowCmd 发布器，可使仿真机器人真正动作。因此默认 `inspect` 只读；`stand/rl` 必须两重显式确认。绝不替换原 FSM，不启动 Z1 官方SDK，不修改 `b2z1_ros2_ws`，不触碰现有 Python V2 baseline。

## 验证层级

1. `test_m15_target_guard`：动作比例（每腿 `[.4,.45,.45]`）、FR/FL/RR/RL↔FL/FR/RL/RR映射、目标关节软限幅、速率限制、NaN/姿态保护。纯数学，无DDS。
2. `--mode inspect`：只读SDK2 LowState，50Hz 构建799维并ONNX推理，无LowCmd发布对象，先确认接收的20槽位来自B2Z1仿真。
3. `--mode stand`：发送200Hz仿真LowCmd，2.5s插值到B2默认站立角，并以PD保持；Z1 6关节**锁定开始时的测量姿态**，不执行IK。初次只能在MuJoCo场景下进行。
4. `--mode rl`：先走2.5s站立插值，再开启50Hz Interactive推理；200Hz 发布腿关节位置目标；Z1仍锁定，0速度指令。

### 固定配置（仿真）

- DDS `ChannelFactory::Init(0,"lo")`，**只连本机仿真**；不能通过CLI选择其他接口。
- Motor feedback/command：SIM `FL,FR,RL,RR,joint1..joint6`，下标`0..17`；数组`18..19`不驱动。此约定仅适用于你确认18 actuator的当前联合MuJoCo桥；SDK2官方B2设备关节顺序不必一致。即使LowState有20槽也**不足以证明**加载的是B2Z1场景，故额外检查Z1关节2/3有效姿态；仍要求操作员肉眼确认场景和接口。
- B2位置PD：`Kp=360, Kd=5`；Z1固定姿态PD：`Kp=90, Kd=6`，只取自先前**Python MuJoCo V2**，**不是实机建议增益**。
- `kStandRampSeconds=2.5`，腿部位置命令最大变化`0.008rad/5ms`，有限仿真关节范围和异常姿态/超时/跟踪误差中止。
- Control loop=200Hz；策略观测/动作=50Hz；固定机身速度0，固定末端目标`[0.45,0,0.35]`在B2底座坐标系中，训练名义安装偏移由原反馈适配层处理。
- 发生错误、Ctrl+C或测试完成时向MuJoCo发送100ms零位置增益/阻尼命令。**这会让仿真机身失去支撑而倒下，并不等于实机急停。** 重启/Reset仿真后再试。操作人应确保没有其他LowCmd控制进程。
- 实机Z1独立SDK尚未接入；此处12..17完全是同一MuJoCo仿真LowState/LowCmd中模拟臂执行器，不得直接用于实机。

## 操作顺序

先退出原 `b2z1_controller`、其他任何发布 `rt/lowcmd` 的进程，保证MuJoCo用你此前已检查的18电机场景在 `lo` 接口独立运行。

编译：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
cmake -S deploy/interactive_b2z1/sim_closed_loop -B deploy/interactive_b2z1/build/sim_closed_loop -DCMAKE_BUILD_TYPE=Debug
cmake --build deploy/interactive_b2z1/build/sim_closed_loop -j2
ctest --test-dir deploy/interactive_b2z1/build/sim_closed_loop --output-on-failure
```

准备：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
export LD_LIBRARY_PATH="/usr/local/lib:$PWD/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib:${LD_LIBRARY_PATH:-}"
BIN=./deploy/interactive_b2z1/build/sim_closed_loop/b2z1_interactive_sim_closed_loop
MODEL=deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx
```

第一轮先只读（绝对不发送LowCmd）：

```bash
"$BIN" "$MODEL" --mode inspect --seconds 5 \
  > deploy/interactive_b2z1/build/m15_inspect.csv
```

**先不要立刻运行 `stand/rl`。** 检查本地的MuJoCo画面和 `m15_inspect.csv`，确认机器人模型直立且Z1第2/3关节反馈真实；还需要检查模拟桥LowCmd关节映射，否则通过长度检查并不等于控制输出正确。

只有明确确认连接的是本机MuJoCo仿真、且没有连接物理机器人或DDS转发后，才在**新的/重置的仿真场景**下测试：

```bash
export B2Z1_SIM_ONLY_ACK=MUJOCO_LOOPBACK_NO_HARDWARE
"$BIN" "$MODEL" --mode stand --seconds 6 --publish-sim-lowcmd \
  > deploy/interactive_b2z1/build/m15_stand.csv
```

观察B2应在2.5秒内逐步进入默认站姿，Z1保持进入时的6关节姿态。机器人会在结束时解除PD并可能倒地，请重置仿真。待 `stand` 单独确认通过，再在重置仿真后执行：

```bash
"$BIN" "$MODEL" --mode rl --seconds 8 --publish-sim-lowcmd \
  > deploy/interactive_b2z1/build/m15_rl.csv
```

进入RL之前先确保没有其它控制进程。**不要把这个命令放到实机或与实机DDS联网的主机上。**

## 验收与失败排查

- 数学单测PASS，`inspect`无LowCmd输出且完整5s退出0。
- `stand` 无明显腿部目标突变，姿态/反馈有效，未触发保护，并能观察到机身维持站立。
- `rl` 在2.5s后动作数值正常，机器人未倒，B2腿部姿态按照策略变化、Z1仍保持锁定，返回码0。
- 若显示 `another LowCmd publisher ...`：先查其他控制进程，不可关闭互斥检测强行控制。
- 若`LowState timeout`、`simulation torso tilt`、`large joint tracking error`：按错误退出，检查仿真姿态、关节映射、起始状态；禁止通过放宽保护掩盖故障。
- 若C++编译报SDK2接口与当前安装版本不一致，直接反馈编译日志。不要为了编译通过修改硬件通信库。
- 即使RL模式通过，也仅说明当前MuJoCo通信式腿部闭环初步可用，距离实机仍有映射、增益和安全FSM独立验收。
