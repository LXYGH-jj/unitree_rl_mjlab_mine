# Go2 ROS2 实机部署说明

本文档只适用于本目录 `deploy_ros2/` 下的 ROS2 版实机部署入口。它与根目录 `README_deploy_zh.md` 中的 SDK2 直连部署方式不同。

## 与原部署方式的区别

原 `deploy/robots/go2` 使用 Unitree SDK2 C++ wrapper：

```text
go2_ctrl --network=<网卡名>
SDK2 ChannelFactory -> rt/lowstate / rt/lowcmd
```

本目录使用 ROS2 topic：

```text
go2_ros2_ctrl
/lowstate -> 策略/FSM -> /lowcmd
/api/motion_switcher/request -> ReleaseMode
```

因此 ROS2 版不使用 `--network` 参数，网卡选择由 CycloneDDS/ROS2 环境变量决定。

## 依赖

需要先有可用的 ROS2 Humble 环境，以及 Unitree ROS2 消息包：

- `rclcpp`
- `unitree_go`
- `unitree_api`
- `yaml-cpp`
- `Eigen3`
- `spdlog`
- `fmt`
- Unitree SDK2 头文件中的 `unitree_joystick.hpp`，仅用于解析手柄数据

当前代码会从以下位置查找 `unitree_go/unitree_api`，实际使用时建议 source 你的 ROS2 工作区：

```bash
source /opt/ros/humble/setup.bash
source /home/vi/lxy/b2z1_ws/code/install/setup.bash
```

实机通信建议使用 CycloneDDS：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

并按 Unitree ROS2 的方式配置 `CYCLONEDDS_URI`，指定连接机器人网口。例如：

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
  <NetworkInterface name="enp5s0" priority="default" multicast="default" />
</Interfaces></General></Domain></CycloneDDS>'
```

其中 `enp5s0` 替换为电脑连接 Go2 的实际网卡名。

## 编译

在仓库根目录执行：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
source /opt/ros/humble/setup.bash
source /home/vi/lxy/b2z1_ws/code/install/setup.bash

cmake -S deploy_ros2 -B build/deploy_ros2
cmake --build build/deploy_ros2 -j$(nproc)
```

也可以显式指定 `CMAKE_PREFIX_PATH`：

```bash
CMAKE_PREFIX_PATH=/home/vi/lxy/b2z1_ws/code/install:/opt/ros/humble \
  cmake -S deploy_ros2 -B build/deploy_ros2
cmake --build build/deploy_ros2 -j$(nproc)
```

产物：

```text
build/deploy_ros2/go2_ros2_ctrl
```

## 策略文件

默认配置在：

```text
deploy_ros2/config/config.yaml
```

其中策略路径为：

```yaml
policy_dir: ../../deploy/robots/go2/config/policy/velocity
```

也就是复用原 Go2 部署目录下的策略文件：

```text
deploy/robots/go2/config/policy/velocity/<版本目录>/
  exported/policy.onnx
  params/deploy.yaml
```

程序会自动选择该目录下带 `exported` 的最新版本目录。

## 运行

先确认 ROS2 能看到 Go2 topic：

```bash
ros2 topic list | grep -E 'lowstate|lowcmd|motion_switcher'
ros2 topic echo /lowstate --once
```

运行控制器：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
source /opt/ros/humble/setup.bash
source /home/vi/lxy/b2z1_ws/code/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

./build/deploy_ros2/go2_ros2_ctrl
```

如需指定另一份配置：

```bash
./build/deploy_ros2/go2_ros2_ctrl --config /path/to/config.yaml
```

## 控制流程

启动后程序会：

1. 订阅 `/lowstate`，等待收到 Go2 低层状态。
2. 向 `/api/motion_switcher/request` 发布数次 `ReleaseMode` 请求。
3. 启动 FSM，默认进入 `Passive`。
4. 手柄按键切换状态：
   - `LT + A`：进入 `FixStand`
   - `RT + X`：从 `FixStand` 进入 `Velocity` 策略控制
   - `LT + B`：返回 `Passive`

## 配置项

`deploy_ros2/config/config.yaml` 中 ROS2 通信相关字段：

```yaml
ROS2:
  lowstate_topic: /lowstate
  lowcmd_topic: /lowcmd
  motion_switch_request_topic: /api/motion_switcher/request
  lowstate_timeout_sec: 1.0
  release_interval_sec: 1.0
  release_max_attempts: 5
```

如果你的 Unitree ROS2 topic 带命名空间，可以在这里改。

## 常见问题

1. `Could not find unitree_goConfig.cmake`

没有 source Unitree ROS2 工作区，或 `CMAKE_PREFIX_PATH` 没包含其 install 路径。执行：

```bash
source /home/vi/lxy/b2z1_ws/code/install/setup.bash
```

2. `ros2 topic echo /lowstate` 没数据

优先检查：

- 机器人和电脑网口 IP 是否在正确网段
- `RMW_IMPLEMENTATION` 是否为 `rmw_cyclonedds_cpp`
- `CYCLONEDDS_URI` 中网卡名是否正确
- 是否已经 source Unitree ROS2 环境

3. 程序一直等待 lowstate

说明没有收到 `/lowstate`。先单独运行：

```bash
ros2 topic echo /lowstate --once
```

如果该命令也无输出，问题在 ROS2/DDS 网络环境，不在策略程序。

4. 策略文件找不到

确认存在：

```text
deploy/robots/go2/config/policy/velocity/<版本目录>/exported/policy.onnx
deploy/robots/go2/config/policy/velocity/<版本目录>/params/deploy.yaml
```

5. 不要和 SDK2 直连控制器同时运行

ROS2 版会发布 `/lowcmd`，原 SDK2 版会发布 `rt/lowcmd`。实机低层控制时不要同时运行多个控制进程。
