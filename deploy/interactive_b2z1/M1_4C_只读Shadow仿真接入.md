# M1.4c — B2Z1 Interactive 独立只读 Shadow（MuJoCo lo）

## 已知基线

本模块接在 M1.4b 反馈适配器之后。799 维 C++ 观测与 TorchScript（M1.1）、ONNX（M1.2）、C++ OrtRunner（M1.3）、25 帧内部策略反馈闭环（M1.4a）此前已分别验证。

**本包仅新增文件，不修改现有 `State_RLBase.cpp`、`CtrlFSM`、`main.cpp` 和 `Z1SdkController`。**

## 架构与安全边界

独立可执行文件 `b2z1_interactive_shadow_sim`：

- **仅连接 DDS `lo`**；没有可选物理网卡入口。
- 只创建 `unitree::robot::go2::subscription::LowState`；不创建 `LowCmd` 发送器，不启用 `CtrlFSM`、`Z1SdkController`、`z1_ctrl`，不调用任何电机控制API。
- 本版本 **仅面向能在 `LowState.motor_state()[0..17]` 中提供全18关节的 MuJoCo B2+Z1 联合仿真**。
- B2 电机 0..11、模拟 Z1 电机 12..17在同一份仿真报文中复制；实机 **禁止** 把 B2 电机 12..17 当作真实 Z1 反馈。
- 采用 M1.4b `adaptFeedback()` 进行顺序映射、IMU归一化、目标坐标及时间检查，再用 M1.4a `InteractivePipeline` 调用现有 C++ `OrtRunner`，更新历史观测。
- 没有实际 EE 目标规划，固定演示输入：速度 `(0,0,0)`，期望EE目标（B2 base）`(0.45,0,0.35)`；这些值只是离线观测输入，不是Z1末端运动命令。
- Z1实际后移安装参数只在未来独立FK/IK中使用，训练观测名义偏置仍是 `[0.20,0,0.09]`。

### 时间戳限制

新的 `SnapshotStore` 是线程安全的反馈快照容器，实际回调使用时应传入**收到原始消息时**采集的本机`steady_clock`时间。当前 `LowState` 轮询封装只公开 `update()/isTimeout()`，所以仿真程序以 `update()` 结束时的本机时间近似表示单帧时间（辅以 `isTimeout()`），**不等于真正的消息接收时间，不能据此声称真实机器人上满足100ms数据新鲜度**。后续接实机必须改为接收回调的时间戳或序号校验。

## 0. 先验证已推送的M1.4b

在工程根目录：

```bash
mkdir -p deploy/interactive_b2z1/build

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Ideploy/interactive_b2z1/include \
  deploy/interactive_b2z1/src/interactive_observation.cpp \
  deploy/interactive_b2z1/src/interactive_feedback_adapter.cpp \
  deploy/interactive_b2z1/tests/test_feedback_adapter.cpp \
  -o deploy/interactive_b2z1/build/test_feedback_adapter
./deploy/interactive_b2z1/build/test_feedback_adapter
```

预期 `M1.4b FEEDBACK ADAPTER: PASS`。如果不通过先修复，再继续。

## 1. 编译并测试SnapshotStore（不连接机器人）

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab

g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
  -Ideploy/interactive_b2z1/include \
  deploy/interactive_b2z1/src/interactive_observation.cpp \
  deploy/interactive_b2z1/src/interactive_feedback_adapter.cpp \
  deploy/interactive_b2z1/src/interactive_snapshot_store.cpp \
  deploy/interactive_b2z1/tests/test_snapshot_store.cpp \
  -o deploy/interactive_b2z1/build/test_snapshot_store

./deploy/interactive_b2z1/build/test_snapshot_store
```

预期 `M1.4c SNAPSHOT STORE: PASS`。

## 2. 单独编译Shadow仿真可执行文件

```bash
cmake -S deploy/interactive_b2z1/shadow_sim \
  -B deploy/interactive_b2z1/build/shadow_sim
cmake --build deploy/interactive_b2z1/build/shadow_sim -j2
```

这个子工程只链接 unitree SDK2、dds 和 ONNX Runtime，不编译原 `b2z1_ctrl` 主程序，不依赖Z1 SDK。你本机已经具备类似B2Z1 SDK2编译环境；但这份新增可执行文件尚未在你本机编译验证。若编译失败，记录完整的CMake及链接错误，避免盲目修改已有控制器。

## 3. MuJoCo中运行Shadow

**终端 A**：使用你之前验证过的完整B2+Z1 18关节MuJoCo场景，在loopback(`--network lo`)启动`unitree_mujoco`。不要开启原`b2z1_ctrl`，不要连接物理网络。

**终端 B**：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
./deploy/interactive_b2z1/build/shadow_sim/b2z1_interactive_shadow_sim \
  deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx 250 \
  | tee deploy/interactive_b2z1/build/shadow_sim_250.csv
```

预期打印：

```text
[shadow_sim] Loopback DDS, waiting for simulated LowState ...
[shadow_sim] Connected. READ ONLY. No motor commands.
frame,b2_q0,sim_z1_q0,leg_action0,max_abs_leg_action,step_ms
0,...
...
[shadow_sim] completed. No hardware commands were sent.
```

程序在报文超时、少于18关节、非法四元数/NaN、ONNX推理异常时**立即停止，不执行恢复性电机动作**。当前仅日志测试，不会令机器人行走或操作机械臂。

## 4. 验收要点

- 使用 `nm -C ... | grep` 可以审计可执行文件，但最重要的依据是代码里根本不存在创建LowCmd以及Z1控制器的路径。
- CSV每帧打印 `b2_q0`, `sim_z1_q0`, `leg_action0`, `max_abs_leg_action`, `step_ms`。确认Z1关节值来自联合MuJoCo，而非恒零填充。
- 实际推理耗时尤其是 `step_ms` 应记录平均值、P95、最大值，后续根据20ms周期做性能评估。
- 若 `sim_z1_q0` 始终为零，未必是错误（Z1关节1零位可为0）；可把打印扩展到其他5个关节或先检查仿真数据来源。
- 本阶段是**通信仿真只读**，不是实机部署验收。

## 5. 下一阶段（本包不包括）

1. Z1 独立且不抢占控制权的真正**只读状态来源**（官方SDK原有构造函数会主动设置`LOWCMD`，不能复用）。
2. DDS回调中的B2接收时间戳，Z1回调时间戳，同步配对与迟到数据拒绝。
3. 完整EE目标世界/机体坐标转换及真实安装 `[0.03,0,0.08]` FK/IK校验。
4. 在另一个独立模块内实现实际低层控制的动作限幅、速率约束、监测与故障处理，**经仿真和硬件安全评审后**再接原FSM和下发通路。

## Git提交

```bash
git status --short
git add deploy/interactive_b2z1/include/interactive_snapshot_store.h \
  deploy/interactive_b2z1/src/interactive_snapshot_store.cpp \
  deploy/interactive_b2z1/tests/test_snapshot_store.cpp \
  deploy/interactive_b2z1/shadow_sim \
  deploy/interactive_b2z1/M1_4C_只读Shadow仿真接入.md
git diff --cached --stat
git diff --cached --check
git commit -m "feat: add simulation-only Interactive shadow subscriber"
git push
```
