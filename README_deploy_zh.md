# Unitree RL Mjlab Go2 部署手册（仅部署，不训练）

本文档用于在另一台电脑上完成 `Go2` 策略部署，不包含训练流程。

## 1. 适用范围

- 目标：运行 `deploy/robots/go2/build/go2_ctrl`，将现成策略部署到实机。
- 不需要：`python scripts/train.py ...`、`pip install -e .` 等训练相关步骤。
- 建议系统：Ubuntu 22.04（x86_64）。

---

## 2. 代码与文件结构

部署至少需要以下目录：

```text
unitree_rl_mjlab/
  deploy/
    include/
    thirdparty/
    robots/go2/
      config/
      src/
      include/
      CMakeLists.txt
```

策略文件放置规则（Go2 速度控制）：

```text
deploy/robots/go2/config/policy/velocity/<版本目录>/
  exported/policy.onnx
  params/deploy.yaml
```

示例：

```text
deploy/robots/go2/config/policy/velocity/v1/exported/policy.onnx
deploy/robots/go2/config/policy/velocity/v1/params/deploy.yaml
```

---

## 3. 系统依赖安装

```bash
sudo apt update
sudo apt install -y \
  cmake g++ build-essential \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```
                 
如果你还要做部署前仿真联调（`simulate/unitree_mujoco`），额外安装：

```bash
sudo apt install -y libglfw3-dev
```

---

## 4. CycloneDDS 安装

建议单独安装一套 CycloneDDS，避免不同来源的 `ddsc/ddscxx` 混用。
如果系统里同时安装了 ROS 2，也要特别注意运行时不要让 ROS 自带的 `libddsc` 抢在你自己的安装前面加载。

推荐目录约定（可按需替换）：
- 源码下载目录：`/home/vi/robot_sdk/cyclonedds`
- 安装目录：`/usr/local`

## 4.1 源码安装（推荐）

```bash
mkdir -p /home/vi/robot_sdk
cd /home/vi/robot_sdk
git clone https://github.com/eclipse-cyclonedds/cyclonedds.git
cd cyclonedds
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)
sudo make install
```

## 4.2 环境变量

```bash
export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

如果你要运行 `b2_ctrl`，通常还需要把 ONNX Runtime 动态库也放进运行时搜索路径：

```bash
export LD_LIBRARY_PATH=/usr/local/lib:/path/to/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib:$LD_LIBRARY_PATH
```

说明：

- 如果当前终端已经 `source /opt/ros/humble/setup.bash`，运行时很容易混入 ROS 自带的 `libddsc`
- 更稳妥的做法是：新开一个终端，不加载 ROS，再手动设置上面这条 `LD_LIBRARY_PATH`
- 如果需要彻底排除 ROS 干扰，可以先执行：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/path/to/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
```

可按需写入 `~/.bashrc`，但如果你日常同时使用 ROS，建议只在运行部署/仿真前临时设置，避免互相污染。

## 4.3 安装检查

```bash
ls /usr/local/lib/libddsc.so
ls /usr/local/lib/libddscxx.so
```

---

## 5. unitree_sdk2 安装与放置

## 5.1 推荐做法（最省事）

推荐目录约定（可按需替换）：
- 源码下载目录：`/home/vi/robot_sdk/unitree_sdk2`
- 安装目录：`/usr/local`

将 `unitree_sdk2` 安装到系统默认路径 `/usr/local`：

```bash
mkdir -p /home/vi/robot_sdk
cd /home/vi/robot_sdk
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir -p build && cd build
cmake ..
sudo make install
```

这样部署编译时通常不需要额外手工传 include/lib 路径。

## 5.2 安装后检查

确认关键文件存在：

```bash
ls /usr/local/include/unitree/dds_wrapper/robots/go2/go2.h
ls /usr/local/include/ddscxx/dds/dds.hpp
ls /usr/local/lib/libddscxx.so
```

---

## 6. 编译 Go2 控制程序

```bash
cd /path/to/unitree_rl_mjlab/deploy/robots/go2
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

产物：

```text
deploy/robots/go2/build/go2_ctrl
```

---

## 7. 准备策略文件（不训练场景）

从已有训练结果（或已有部署包）拷贝 `policy.onnx`：

```bash
cd /path/to/unitree_rl_mjlab
mkdir -p deploy/robots/go2/config/policy/velocity/v1/exported
mkdir -p deploy/robots/go2/config/policy/velocity/v1/params

# 复制策略
cp /your/path/policy.onnx deploy/robots/go2/config/policy/velocity/v1/exported/

# 复制一份 deploy 参数模板（可再按需调整）
cp deploy/robots/go2/config/policy/velocity/v0/params/deploy.yaml \
   deploy/robots/go2/config/policy/velocity/v1/params/deploy.yaml
```

`go2` 默认配置里 `policy_dir` 为 `config/policy/velocity`，程序会自动在该目录下选择带 `exported` 的版本目录。

---

## 8. 实机网络准备

1. 机器人上电，进入可调试状态。  
2. 网线直连电脑与机器人。  
3. 配置电脑网卡到机器人网段（示例）：
   - IP：`192.168.123.222`
   - 掩码：`255.255.255.0`
4. 查询网卡名：

```bash
ip -br addr
```

例如得到 `enp5s0`。

---

## 9. 运行实机控制

```bash
cd /path/to/unitree_rl_mjlab/deploy/robots/go2/build
./go2_ctrl --network=enp5s0
```

`--network` 填写你的机器人连接网卡名。

---

## 10. 实机安全回退流程

如果最终部署的是带策略控制的四足（例如 `b2_ctrl`），建议在正式上机器人前，先确认**人工安全回退链**可用。

### 10.1 设计原则

- 不建议从 `Velocity` 直接切到完全卸力状态
- 当前 B2 部署建议采用：**先退出策略，直接进入受控卧倒，再卸力**
- 这样可以避免策略仍在输出动作时突然失去支撑，同时减少“先锁站再卧倒”的额外切换

推荐流程：

```text
Velocity -> LieDown -> Passive
```

### 10.2 B2 推荐按键流程

对于当前 `b2` 部署配置，推荐使用下面这条两步式人工确认回退链：

- `RT + B`：`Velocity -> LieDown`
- 等待机体稳定落地
- `LT + B`：`LieDown -> Passive`

如果你当前使用的是键盘映射，对应关系为：

- `Ctrl + K`：`Velocity -> LieDown`
- `Shift + K`：`LieDown -> Passive`

### 10.3 现场操作建议

第一次实机验证时，建议遵循以下约束：

- 使用吊装、旁扶或软地面保护
- 首次进入 `Velocity` 前，先单独验证 `FixStand`
- 首次验证策略时只给很小的速度指令
- 操作人员始终站在可立即执行回退按键的位置
- 未确认 `Velocity -> LieDown -> Passive` 全链路可用前，不做大幅运动

### 10.4 为什么不能直接从策略切到 Passive

因为 `Passive` 本质上是低增益/卸力状态，如果机器人仍处于策略动态控制过程中：

- 支撑力会突然消失
- 身体可能瞬间塌落
- 落地冲击会传到髋、膝和小腿关节

这里不直接切到 `Passive`，而是先进入 `LieDown`，用慢轨迹把机体有控制地放到地面。

### 10.5 建议的现场验收顺序

在真正让机器人行走前，建议按下面顺序逐项验收：

1. `Passive -> FixStand` 可稳定站起  
2. `FixStand -> LieDown` 可平稳卧倒  
3. `LieDown -> Passive` 可正常卸力  
4. `Velocity -> LieDown` 可在策略运行中可靠退出  
5. 完整链路 `Velocity -> LieDown -> Passive` 可重复执行  

只有这五项都通过后，再进入较长时间的策略行走测试。

---

## 11. B2 调试数据导出与离线分析

如果你希望在仿真验证部署或实机测试时记录 `observation` 和 `action`，并在事后离线分析，可使用当前 B2 部署链里已经接好的 `debug_logger`。

### 11.1 打开 CSV 记录

编辑 [deploy/robots/b2/config/policy/velocity/v0/params/deploy.yaml](/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2/config/policy/velocity/v0/params/deploy.yaml:1)，将下面配置打开：

```yaml
debug_logger:
  enabled: true
  print_to_console: true
  write_csv: true
  every_n_steps: 10
  csv_path: log/b2_velocity_debug.csv
```

说明：

- `enabled`：是否启用调试日志
- `print_to_console`：是否在终端打印 observation/action
- `write_csv`：是否写入 CSV
- `every_n_steps`：每隔多少个控制周期记录一次
- `csv_path`：CSV 输出路径，相对路径默认相对于 `deploy/robots/b2/`

生成文件示例：

```text
deploy/robots/b2/log/b2_velocity_debug.csv
```

说明：

- 仿真联调和实机测试都会生成同样结构的 CSV
- 因此后续离线分析脚本和 `wandb` 上传流程可以直接复用
- 实机中的 `base_linear_velocity`、`base_angular_velocity`、`projected_gravity` 来自状态估计与传感器反馈，通常会比仿真更噪一些

### 11.2 运行离线分析脚本

记录完 CSV 后，可运行下面的脚本生成离线报告、曲线图和列语义说明：

```bash
cd /path/to/unitree_rl_mjlab
python3 scripts/analyze_b2_velocity_debug.py \
  --input deploy/robots/b2/log/b2_velocity_debug.csv \
  --output-dir deploy/robots/b2/log/b2_velocity_debug_analysis
```

输出内容包括：

- `report.html`：可直接打开的离线分析报告
- `column_semantics.csv`：每一列对应的实际物理意义
- `column_semantics.json`：列语义的 JSON 版本
- `summary_stats.json`：各变量的最小值、最大值、均值
- `*.svg`：各类曲线图

生成目录示例：

```text
deploy/robots/b2/log/b2_velocity_debug_analysis/
```

如果你是在实机测试后处理数据，也可以直接换一个输出目录，例如：

```bash
cd /path/to/unitree_rl_mjlab
python3 scripts/analyze_b2_velocity_debug.py \
  --input deploy/robots/b2/log/b2_velocity_debug.csv \
  --output-dir deploy/robots/b2/log/b2_velocity_debug_analysis_real
```

### 11.3 各项数据的物理意义

当前脚本会把 CSV 列还原成带物理语义的名字，主要包括：

- `base_vx/base_vy/base_vz`：机体坐标系下的线速度，分别对应前向、左向、上向，单位 `m/s`
- `base_wx/base_wy/base_wz`：机体坐标系下的角速度，分别对应滚转、俯仰、偏航，单位 `rad/s`
- `gravity_x_b/gravity_y_b/gravity_z_b`：重力在机体系下的投影，可用于判断机体是否接近直立，单位无量纲
- `FL/FR/RL/RR_*_pos_rel`：各腿关节相对默认站姿的角度偏差，单位 `rad`
- `FL/FR/RL/RR_*_vel`：各腿关节角速度，单位 `rad/s`
- `FL/FR/RL/RR_*_raw_action`：策略网络原始输出，尚未经过缩放、偏置和裁剪，单位无量纲
- `FL/FR/RL/RR_*_target_pos`：真正下发给低层 PD 的关节目标位置，单位 `rad`
- `cmd_vx/cmd_vy/cmd_wz`：遥操作输入的前向、侧向、偏航速度指令，单位分别为 `m/s`、`m/s`、`rad/s`

说明：

- 这里的腿顺序按 `FL, FR, RL, RR`
- 每条腿内部关节顺序按 `hip, thigh, calf`
- 这些语义会写进 `column_semantics.csv` 和 `report.html`

### 11.4 可选：离线写入 W&B

如果你希望把分析结果作为离线 run/artifact 写入 `wandb`，可执行：

```bash
cd /path/to/unitree_rl_mjlab
python3 scripts/analyze_b2_velocity_debug.py \
  --input deploy/robots/b2/log/b2_velocity_debug.csv \
  --output-dir deploy/robots/b2/log/b2_velocity_debug_analysis_wandb \
  --wandb-project b2-debug \
  --wandb-run-name b2-velocity-debug
```

如果是实机测试数据，建议单独命名 run，方便和仿真区分，例如：

```bash
cd /path/to/unitree_rl_mjlab
python3 scripts/analyze_b2_velocity_debug.py \
  --input deploy/robots/b2/log/b2_velocity_debug.csv \
  --output-dir deploy/robots/b2/log/b2_velocity_debug_analysis_real_wandb \
  --wandb-project b2-debug \
  --wandb-run-name b2-velocity-real
```

如果运行机器允许 `wandb` 启动本地服务，后续可再同步：

```bash
wandb sync deploy/robots/b2/log/b2_velocity_debug_analysis_wandb/wandb_offline/wandb/offline-run-*
```

说明：

- `wandb` 页面中的 `interactive/...` 图表是可交互折线图，可以用鼠标悬停查看每个点的时间、曲线名和数值
- `offline_report_html` 仍会保留在 run 中作为辅助报告，但它不是训练式交互图
- 如果当前环境是受限沙箱，`wandb` 可能起不来；这种情况下本地 HTML/SVG/CSV 报告仍会正常生成

---

## 12. 常见问题排查

1. `go2/build` 不存在  
原因：还没编译。执行第 6 节。

2. `policy.onnx` 找不到  
原因：策略目录缺少 `exported/policy.onnx`。执行第 7 节。

3. `libddscxx.so.0 => not found` 或 `libonnxruntime.so.1 => not found`  
可先临时补库路径：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/path/to/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
```

4. `fatal error: unitree/dds_wrapper/...`  
原因：`unitree_sdk2` 头文件未安装到标准搜索路径。重新执行第 5 节安装。

5. 运行时 DDS 断言或异常（常见于 `dds_write...`）、`free(): invalid pointer`、`corrupted size vs. prev_size`  
原因：进程加载了不同来源的 `libddsc`/`libddscxx`，最常见的是 ROS 的 `libddsc` 和 `/usr/local` 的 `libddscxx` 被混用。  
先检查：

```bash
ldd deploy/robots/b2/build/b2_ctrl | grep -E 'libddsc|libddscxx|libonnxruntime'
ldd simulate/build/unitree_mujoco | grep -E 'libddsc|libddscxx'
```

确保 `libddsc` 和 `libddscxx` 来自同一套安装路径，例如都来自 `/usr/local/lib`，再运行。

6. `The other process is using the lowcmd channel`  
原因：已有其他控制进程占用。关闭旧进程后重试。

7. 跨机器复制工程后，`cmake ..` 提示 `CMakeCache.txt` 路径不一致  
原因：`build/` 目录里保留了旧机器生成的 CMake 缓存，缓存中的源码路径和当前机器路径不同。  
处理方法：删除旧 `build` 后重新配置。

```bash
cd /path/to/unitree_rl_mjlab/deploy/robots/b2
rm -rf build
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

如果你还要编译 `simulate` 或其他机器人目录，也建议同样先清理各自的 `build/` 目录，再重新运行 `cmake ..`。

---

## 13. 可选：部署前仿真联调

如果需要先联调：

1. 终端 A：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib
cd /path/to/unitree_rl_mjlab/simulate/build
./unitree_mujoco
```

2. 终端 B：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/path/to/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
cd /path/to/unitree_rl_mjlab/deploy/robots/go2/build
./go2_ctrl --network=lo
```

`lo` 仅用于本机仿真联调；实机必须使用真实网卡名。

建议在启动前做一次运行时库自检：

```bash
ldd simulate/build/unitree_mujoco | grep -E 'libddsc|libddscxx'
ldd deploy/robots/b2/build/b2_ctrl | grep -E 'libddsc|libddscxx|libonnxruntime'
```
