# 安装配置文档

## 系统要求

- **操作系统**：推荐使用 Ubuntu 22.04
- **显卡**：Nvidia 显卡  
- **驱动版本**：建议使用 550 或更高版本  

---

## 1. 创建虚拟环境

建议在虚拟环境中运行训练或部署程序，推荐使用 Conda 创建虚拟环境。如果您的系统中已经安装了 Conda，可以跳过步骤 1.1。

### 1.1 下载并安装 MiniConda

MiniConda 是 Conda 的轻量级发行版，适用于创建和管理虚拟环境。使用以下命令下载并安装：

```bash
mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm ~/miniconda3/miniconda.sh
```

安装完成后，初始化 Conda：

```bash
~/miniconda3/bin/conda init --all
source ~/.bashrc
```

### 1.2 创建新环境

使用以下命令创建虚拟环境：

```bash
conda create -n unitree_rl_mjlab python=3.11
```


### 1.3 激活虚拟环境

```bash
conda activate unitree_rl_mjlab
```

---

## 2. 安装

### 2.1 下载

通过 Git 克隆仓库：

```bash
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git
```

### 2.2 安装依赖

```bash
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
```

我们将其余所需依赖放入 setup.py 文件中，
进入 unitree_rl_mjlab 项目根目录并安装：

```bash
cd unitree_rl_mjlab
pip install -e .
```

## 总结

按照上述步骤完成后，您已经准备好在虚拟环境中运行相关程序。若遇到问题，请参考各组件的官方文档或检查依赖安装是否正确。

---

## 部署仿真地形切换

部署仿真器默认加载平地场景，配置在：

`simulate/config.yaml`

其中 `robot_scene` 字段决定加载哪个 MuJoCo 场景 XML。除了默认平地，现在仓库里也提供了楼梯、斜坡和离散障碍块场景。

例如，B2Z1 可以切换为：

```yaml
robot_scene: "src/assets/robots/unitree_b2z1/xmls/scene_b2z1_stairs.xml"
```

或者：

```yaml
robot_scene: "src/assets/robots/unitree_b2z1/xmls/scene_b2z1_slope.xml"
```

也可以在启动仿真时直接指定：

```bash
cd simulate/build
./unitree_mujoco -n lo -s src/assets/robots/unitree_b2z1/xmls/scene_b2z1_rough_blocks.xml
```

当前提供的部署仿真地形包括：

- B2:
  `scene_b2.xml`
  `scene_b2_stairs.xml`
  `scene_b2_slope.xml`
  `scene_b2_rough_blocks.xml`
- B2Z1:
  `scene_b2z1.xml`
  `scene_b2z1_stairs.xml`
  `scene_b2z1_slope.xml`
  `scene_b2z1_rough_blocks.xml`

### 常见问题：`dds_writecdr_impl_common` 断言失败

如果仿真场景已经打印出机器人 link / joint / sensor 信息，随后又出现：

```text
dds_writecdr_impl_common: Assertion '(wr->m_iox_pub == NULL) == (d->a.iox_chunk == NULL)' failed.
```

这通常不是地形 XML 的问题，而是 `libddsc` 和 `libddscxx` 来自不同安装路径，导致 CycloneDDS / iceoryx 运行时不一致。

建议在启动仿真器和控制器前，先在同一个终端里固定 DDS 动态库路径：

```bash
export UNITREE_DDS_LIB=/home/vi/lxy/b2+z1/b2z1_hw/external/unitree_sdk2/x86_64/lib
export LD_LIBRARY_PATH=$UNITREE_DDS_LIB:$LD_LIBRARY_PATH
```

然后检查 `unitree_mujoco` 实际加载的是不是同一套库：

```bash
ldd ./unitree_mujoco | grep -E 'libddsc|libddscxx'
```

理想情况是两者都指向同一个目录。确认后再重新启动，例如：

```bash
cd simulate/build
./unitree_mujoco -n lo -s src/assets/robots/unitree_b2z1/xmls/scene_b2z1_stairs.xml
```
