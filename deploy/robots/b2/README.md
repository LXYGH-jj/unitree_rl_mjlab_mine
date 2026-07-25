## deploy

和 Go2 保持一致，就不要指定 /opt/unitree_robotics。用默认 /usr/local 安装

```bash
sudo apt update
sudo apt install -y cmake g++ build-essential libyaml-cpp-dev libeigen3-dev libboost-all-dev libspdlog-dev libfmt-dev

cd /home/svi/deploy_b2/unitree_sdk2
rm -rf build
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF
make -j$(nproc)
sudo make install
sudo ldconfig
```
CMakeLists.txt 里没有指定 /opt/unitree_robotics，它用的是默认安装路径风格，里面还硬编码了：`/usr/local/include/ddscxx`

然后编译b2

```bash
cd /home/svi/deploy_b2/unitree_rl_mjlab/deploy/robots/b2
rm -rf build
mkdir build && cd build
cmake ..
make -j$(nproc)
```
按 Go2 相同方式，unitree_sdk2 默认装到 /usr/local，不加 -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics。




# simulate

## 编译仿真器

```bash
sudo apt install -y libglfw3-dev

cd /home/svi/deploy_b2/unitree_rl_mjlab/simulate
rm -rf build
mkdir build && cd build
cmake ..
make -j$(nproc)
```

## 终端 1：启动 B2 仿真

```bash
cd /home/svi/deploy_b2/unitree_rl_mjlab/simulate/build
./unitree_mujoco -i 0 -n lo
```

如果想在部署仿真里切换不同地形，可以改 `simulate/config.yaml` 里的 `robot_scene`，或者启动时直接覆盖：

```bash
./unitree_mujoco -i 0 -n lo \
  -s src/assets/robots/unitree_b2/xmls/scene_b2_stairs.xml
```

当前提供的 B2 场景有：

- `src/assets/robots/unitree_b2/xmls/scene_b2.xml`
- `src/assets/robots/unitree_b2/xmls/scene_b2_stairs.xml`
- `src/assets/robots/unitree_b2/xmls/scene_b2_slope.xml`
- `src/assets/robots/unitree_b2/xmls/scene_b2_rough_blocks.xml`

## 终端 2：启动 B2 控制器：

```bash
cd /home/svi/deploy_b2/unitree_rl_mjlab/deploy/robots/b2/build
./b2_ctrl -n lo
```

## 进入控制模式：手柄

LT + up      进入 FixStand
RT + A       进入 Velocity/RL 控制
LT + B       退回 Passive`

如果没有手柄，可以先把 /home/svi/deploy_b2/unitree_rl_mjlab/simulate/config.yaml 里的：

`use_joystick: 1`

改成：

`use_joystick: 0`

w/s   前进 / 后退
a/d   左移 / 右移
q/e   左转 / 右转


## 没有添加摔倒保护，直接卸力
