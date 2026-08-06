# Unitree Go2 视觉强化学习部署上下文

> 用途：供 VS Code / Codex 在后续代码调试中快速理解当前硬件环境、ROS2 网络、RealSense 相机配置、已完成工作和下一步任务。  
> 更新时间：2026-08-04

---

## 1. 项目目标

在 Unitree Go2 上部署已训练的视觉行走强化学习策略。

总体架构：

```text
Intel RealSense D435i
        │ USB 3.x
        ▼
Go2 NX 扩展坞
Ubuntu 20.04 ARM64 / ROS2 Foxy
运行 realsense2_camera
        │
        │ ROS2 DDS（CycloneDDS）
        ▼
Ubuntu PC
ROS2 Humble
运行视觉预处理、策略推理和机器人控制程序
        │
        ▼
Unitree Go2 控制接口
```

控制程序计划运行在 PC 上，深度相机物理连接在 Go2 扩展坞上。扩展坞负责发布相机 ROS2 话题，PC 通过有线网络订阅相机和机器人状态，并发布控制指令。

---

## 2. 硬件与系统环境

### 2.1 Go2 扩展坞

```text
设备：Unitree Go2 NX docking station
系统：Ubuntu 20.04
架构：aarch64 / ARM64
平台：NVIDIA Tegra / Jetson
内核：5.10.104-tegra
ROS2：Foxy
```

扩展坞支持：

```text
Bus 02：USB 3.x root hub，10000M
Bus 01：USB 2.0 root hub，480M
```

### 2.2 深度相机

```text
型号：Intel RealSense D435i
USB ID：8086:0b3a
序列号：419622072041
固件版本：05.15.01.55
```

### 2.3 PC

已知环境：

```text
Ubuntu PC
ROS2 Humble
与 Go2 使用网线连接
PC 端之前已使用 CycloneDDS 订阅 Go2 本体话题
```

PC 端 Unitree ROS2 工作空间：

```text
/home/liu/b2z1_ros2_ws/src/unitree_ros2
```

---

## 3. ROS2 与 DDS 配置背景

Go2 本体、扩展坞和 PC 之间的 ROS2 通信建议统一使用 CycloneDDS。

已验证：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

PC 端曾通过 CycloneDDS 成功订阅：

```text
/lowstate
/lf/lowstate
/sportmodestate
```

`/lowstate` 类型：

```text
unitree_go/msg/LowState
```

已观察到 `/lowstate` 包含：

```text
IMU 四元数
角速度
加速度
RPY
motor_state
关节位置
关节速度
关节力矩等
```

PC 端曾使用 `CYCLONEDDS_URI` 绑定有线网卡 `eno1`。

后续相机通信也应沿用同一 DDS 设置。

---

## 4. 扩展坞 ROS 软件源修复

扩展坞原 ROS 源文件：

```text
/etc/apt/sources.list.d/ros-fish.list
```

原问题：

```text
EXPKEYSIG F42ED6FBAB17C654 Open Robotics
```

已更新 ROS key：

```bash
sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

当前 ROS 源配置使用 `signed-by`：

```text
deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main
deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main
```

ROS2 源改为官方源，是因为清华镜像中的 ARM64 Foxy `xacro` 包出现索引存在但 `.deb` 文件 404 的问题。

注意：

```text
不要执行 sudo apt upgrade
不要执行 sudo apt autoremove
```

扩展坞属于 Unitree 定制系统，大规模升级可能破坏其 ROS、DDS 或硬件环境。

---

## 5. RealSense 软件环境

### 5.1 系统原有库

系统原先安装：

```text
ros-noetic-librealsense2 2.50.0
```

位置：

```text
/opt/ros/noetic/lib/aarch64-linux-gnu/
```

CMake 配置：

```text
/opt/ros/noetic/lib/aarch64-linux-gnu/cmake/realsense2/
```

这个 ROS1/Noetic 路径下的版本不直接用于当前 ROS2 wrapper，因为版本和环境容易冲突。

不要卸载该包，以免影响 Unitree 原始环境。

### 5.2 新安装的 librealsense2

为 ROS2 Foxy 单独从源码安装：

```text
librealsense2 2.51.1
```

源码目录：

```text
/home/unitree/librealsense
```

安装位置：

```text
/usr/local/lib/librealsense2.so
/usr/local/lib/librealsense2.so.2.51
/usr/local/lib/librealsense2.so.2.51.1
/usr/local/include/librealsense2/
/usr/local/lib/cmake/realsense2/
```

关键 CMake 文件：

```text
/usr/local/lib/cmake/realsense2/realsense2Config.cmake
/usr/local/lib/cmake/realsense2/realsense2ConfigVersion.cmake
```

源码版本：

```text
v2.51.1
```

构建配置：

```bash
cd ~/librealsense
mkdir -p build
cd build

cmake .. \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false

make -j2
sudo make install
sudo ldconfig
```

### 5.3 realsense-ros

ROS2 工作空间：

```text
/home/unitree/realsense_ws
```

源码：

```text
/home/unitree/realsense_ws/src/realsense-ros
```

使用版本：

```text
realsense-ros 4.51.1
```

不能使用最初下载的 `4.58.3`，因为该版本要求：

```text
librealsense2 >= 2.58.0
```

而当前稳定组合为：

```text
realsense-ros 4.51.1
librealsense2 2.51.1
ROS2 Foxy
```

依赖：

```bash
sudo apt install ros-foxy-xacro
```

构建命令：

```bash
cd ~/realsense_ws
rm -rf build install log

source /opt/ros/foxy/setup.bash

colcon build \
  --parallel-workers 2 \
  --cmake-args \
  -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
```

构建成功结果：

```text
realsense2_camera_msgs：成功
realsense2_description：成功
realsense2_camera：成功
```

---

## 6. USB 3.x 问题与解决结果

### 6.1 原问题

D435i 最初虽然使用 USB 3.0 数据线，但扩展坞 Type-A 口实际枚举为 USB2：

```text
Device USB type: 2.1
lsusb -t 显示 480M
bcdUSB 2.10
```

导致本机 ROS2 图像频率异常：

```text
彩色图像约 1.7 Hz
深度图像约 5 Hz
```

### 6.2 官方修复

按照 Unitree 官方文档：

```text
https://support.unitree.com/home/zh/developer/module_update
```

执行“解决拓展坞上 USB Type-A 口被识别为 USB2.0 的问题”。

升级脚本目录：

```text
/home/unitree/unitree_dt_update
```

执行：

```bash
cd ~/unitree_dt_update
chmod +x unitree_update.sh
sudo ./unitree_update.sh
```

脚本检测：

```text
Found Go 2 NX docking station
Current device tree version: Sep 30 2025 17:55:09
Target device tree version: Sep 30 2025 17:55:09
Device tree version is correct, no update needed
```

重新完整重启/重新枚举后，Type-A 和 Type-C 均已进入 USB3：

```text
lsusb -t
RealSense interfaces：5000M
USB root hub：10000M
```

RealSense 启动日志：

```text
Device USB type: 3.2
```

因此 USB Type-A 修复已经生效。

---

## 7. 当前相机运行状态

加载环境：

```bash
source /opt/ros/foxy/setup.bash
source ~/realsense_ws/install/setup.bash
```

启动：

```bash
ros2 launch realsense2_camera rs_launch.py
```

当前默认配置：

```text
Depth：848 × 480，Z16，30 FPS
Color：1280 × 720，RGB8，30 FPS
Gyro：200 Hz
Accel：100 Hz
```

成功日志：

```text
RealSense ROS v4.51.1
Built with LibRealSense v2.51.1
Running with LibRealSense v2.51.1
Device USB type: 3.2
RealSense Node Is Up!
```

深度话题本机实测：

```text
/camera/depth/image_rect_raw
average rate ≈ 25.6 Hz
```

这个结果可认为正常，可以进入 PC 端通信和策略接入阶段。

### 7.1 启动阶段可忽略的日志

以下参数读取错误发生在传感器初始化阶段，但不会阻止数据流启动：

```text
depth_module.auto_exposure_limit: HW not ready
depth_module.auto_gain_limit: HW not ready
depth_module.auto_exposure_limit_toggle: HW not ready
depth_module.auto_gain_limit_toggle: HW not ready
```

以下警告也未影响节点运行：

```text
rgb_camera.power_line_frequency 默认值 3 超出范围 [0, 2]
```

只有在实际图像曝光或频闪出现问题时，再针对这些参数处理。

---

## 8. 当前 ROS2 相机话题

已知核心话题：

```text
/camera/color/camera_info
/camera/color/image_raw
/camera/color/metadata
/camera/depth/camera_info
/camera/depth/image_rect_raw
/camera/depth/metadata
/camera/extrinsics/depth_to_color
/camera/imu
```

检查命令：

```bash
ros2 node list
ros2 topic list | grep camera
ros2 topic info /camera/depth/image_rect_raw -v
ros2 topic hz /camera/depth/image_rect_raw
ros2 topic hz /camera/color/image_raw
```

---

## 9. 每次新终端必须加载的环境

扩展坞新终端中，若出现：

```text
Package 'realsense2_camera' not found
```

说明没有加载工作空间。

执行：

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/realsense_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

可以考虑加入 `~/.bashrc`，但需要确认 source 顺序不会污染其他 Unitree 程序。

推荐顺序：

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/realsense_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

不要 source：

```text
/opt/ros/noetic/setup.bash
```

否则会把 ROS1 Catkin 路径混入 ROS2 Foxy，引发构建错误，例如：

```text
ROS_DISTRO was set to 'foxy' before
Please make sure that the environment does not mix paths from different distributions
```

---

## 10. 下一阶段任务

### 10.1 第一目标：PC 订阅扩展坞相机话题

扩展坞：

```bash
source /opt/ros/foxy/setup.bash
source ~/cyclonedds_ws/install/setup.bash
source ~/realsense_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 launch realsense2_camera rs_launch.py
```

PC：

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0

ros2 topic list | grep camera
ros2 topic hz /camera/depth/image_rect_raw
```

预期 PC 可见：

```text
/camera/depth/image_rect_raw
/camera/color/image_raw
```

### 10.2 网络与 CycloneDDS 检查

PC 与扩展坞必须：

```text
处于同一有线局域网
IP 可互相 ping
ROS_DOMAIN_ID 一致
RMW_IMPLEMENTATION 均为 rmw_cyclonedds_cpp
CycloneDDS 绑定到正确的有线网卡
```

PC 网卡曾使用：

```text
eno1
```

需要检查当前实际名称：

```bash
ip addr
ip route
```

如需 CycloneDDS XML，应避免绑定 Wi-Fi、Docker、VPN 或 loopback。

### 10.3 图像可视化

在 PC 上可以使用：

```bash
rqt_image_view
```

或：

```bash
rviz2
```

深度图话题：

```text
/camera/depth/image_rect_raw
```

若 RViz/rqt 中没有可选 topic，先检查：

```bash
ros2 topic list
ros2 topic info /camera/depth/image_rect_raw -v
```

以及 QoS 是否匹配。

### 10.4 策略接入

视觉策略节点预期流程：

```text
/camera/depth/image_rect_raw
        ↓
ROS2 sensor_msgs/msg/Image
        ↓
cv_bridge / 自定义零拷贝转换
        ↓
裁剪、缩放、归一化、无效深度处理
        ↓
转换成模型输入 tensor
        ↓
视觉编码器 / CNN
        ↓
策略网络
        ↓
关节目标或控制动作
        ↓
Unitree 控制接口
```

需要明确并验证以下训练—部署一致性：

```text
深度图分辨率
相机内参
深度单位
最大/最小深度截断
无效深度值处理
图像裁剪区域
resize 插值方式
归一化范围
图像帧率
图像延迟
相机安装位姿
策略控制频率
观测历史长度
动作缩放
关节顺序
默认关节角
```

---

## 11. 建议的代码模块划分

建议在 PC 端项目中使用如下结构：

```text
go2_visual_policy/
├── README.md
├── config/
│   ├── camera.yaml
│   ├── cyclone_dds.xml
│   └── policy.yaml
├── scripts/
│   ├── check_topics.sh
│   ├── start_camera_remote.sh
│   └── run_policy.sh
├── src/
│   └── go2_visual_policy/
│       ├── camera_subscriber.py
│       ├── depth_preprocessor.py
│       ├── policy_runner.py
│       ├── robot_state_subscriber.py
│       ├── action_publisher.py
│       ├── safety_monitor.py
│       └── diagnostics.py
├── models/
│   └── policy.pt
└── tests/
    ├── test_depth_preprocessing.py
    ├── test_observation_layout.py
    └── test_action_scaling.py
```

职责建议：

```text
camera_subscriber.py
    只负责 ROS2 图像接收、时间戳和 QoS

depth_preprocessor.py
    只负责深度图裁剪、resize、归一化、无效值处理

robot_state_subscriber.py
    订阅 /lowstate 或其他状态话题，转换成策略本体观测

policy_runner.py
    加载 TorchScript/ONNX/PyTorch 模型并执行推理

action_publisher.py
    将策略输出映射为 Unitree 控制命令

safety_monitor.py
    处理通信超时、图像超时、姿态异常、关节越界和急停

diagnostics.py
    记录相机频率、网络延迟、推理耗时、控制频率和丢帧
```

---

## 12. 代码调试时优先加入的诊断量

每帧建议记录：

```text
camera_header_stamp
camera_receive_time
camera_transport_latency
depth_frame_interval
robot_state_stamp
robot_state_age
preprocess_time
policy_inference_time
action_publish_time
control_loop_period
dropped_camera_frames
stale_observation_count
```

建议先在不发送控制指令的“只读模式”下完成：

```text
相机订阅
状态订阅
观测拼接
模型推理
动作日志
```

确认无误后，再解锁机器人动作发布。

---

## 13. 安全要求

首次实机调试必须：

```text
机器人吊装或由保护架支撑
准备实体急停
限制动作幅值
限制关节目标变化率
增加通信超时保护
相机/状态数据超时立即停止动作
先运行站立策略，再切换视觉行走策略
避免直接以训练最大速度运行
```

任何以下情况都应进入安全模式：

```text
深度图超过阈值时间未更新
/lowstate 超时
策略推理异常或产生 NaN/Inf
机器人姿态超过限制
关节目标越界
控制循环频率明显下降
DDS 通信断开
```

---

## 14. 已知不要重复尝试的错误路线

### 14.1 不要让 ROS2 直接使用整个 `/opt/ros/noetic` 前缀

错误示例：

```bash
-DCMAKE_PREFIX_PATH=/opt/ros/noetic
```

会引入 ROS1 Catkin、genmsg、nav_msgs 等依赖，造成 ROS1/ROS2 环境混用。

### 14.2 不要继续使用 realsense-ros 4.58.3

该版本要求：

```text
librealsense >= 2.58.0
```

当前稳定组合已经是：

```text
realsense-ros 4.51.1
librealsense 2.51.1
```

### 14.3 不要把 USB3 数据线等同于 USB3 枚举

必须用：

```bash
lsusb -t
```

确认设备接口显示：

```text
5000M
```

并通过 RealSense 日志确认：

```text
Device USB type: 3.2
```

### 14.4 不要在没有 source 工作空间时启动相机

新终端必须加载：

```bash
source ~/realsense_ws/install/setup.bash
```

---

## 15. 当前里程碑

```text
[完成] NoMachine 连接扩展坞
[完成] 扩展坞互联网连接
[完成] ROS apt key 修复
[完成] Foxy ARM64 xacro 安装
[完成] D435i USB 识别
[完成] librealsense2 2.51.1 源码编译和独立安装
[完成] realsense-ros 4.51.1 编译
[完成] USB Type-A 从 USB2 修复为 USB3
[完成] RealSense 节点启动
[完成] 深度流本机约 25.6 Hz
[完成] Go2 本体 ROS2 话题通过 CycloneDDS 在 PC 可见
[下一步] PC 端发现并订阅 /camera/* 话题
[下一步] PC 端验证图像频率、延迟和 QoS
[下一步] 对接视觉策略输入
[下一步] 对接 Unitree 状态与动作接口
[下一步] 建立安全保护和诊断
[下一步] 仿真/只读验证后进行实机低风险测试
```

---

## 16. Codex 下一步执行建议

Codex 在继续工作时，优先完成以下任务：

1. 检查 PC 端现有项目目录和策略入口。
2. 找出策略模型格式：PyTorch、TorchScript、ONNX 或其他。
3. 找出训练时深度观测预处理代码。
4. 建立 ROS2 深度图订阅节点。
5. 将部署预处理逐项对齐训练代码。
6. 建立 `/lowstate` 订阅和观测拼接。
7. 建立仅推理、不发布动作的验证模式。
8. 输出实时诊断：相机 FPS、消息延迟、推理耗时。
9. 最后接入 Unitree 动作发布和安全保护。

在修改代码前，必须先确认：

```text
策略模型路径
策略输入 tensor shape
深度图预处理流程
本体观测顺序
历史帧数量
动作维度和关节顺序
动作缩放方式
控制周期
目标 Unitree 控制 topic/API
```
