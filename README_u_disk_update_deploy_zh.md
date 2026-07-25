# U 盘更新 Deploy 恢复流程

本文档用于记录一种不依赖 GitHub、通过 U 盘从另一台电脑拷贝 `deploy/` 或相关文件后，在当前机器上恢复可编译、可运行状态的推荐流程。

适用场景：

- 当前机器不方便直接用 GitHub 更新
- 通过 U 盘、移动硬盘或局域网手工拷贝代码
- 重点针对 `deploy/`、`simulate/`、`policy.onnx`、`config.yaml` 这类变动

---

## 1. 拷贝时的建议

不要用容易丢失权限或软链接信息的方式覆盖目录。

推荐：

```bash
rsync -a /media/your_usb/unitree_rl_mjlab/deploy/ /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/
```

或者：

```bash
cp -a /media/your_usb/unitree_rl_mjlab/deploy/. /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/
```

这样可以尽量保留：

- 可执行权限
- 软链接
- 时间戳
- 目录结构

不要只图省事直接拖拽覆盖，尤其是 `build/`、`exported/`、软链接文件和可执行文件。

---

## 2. 更新后第一步：删除旧 build

只要替换过 `deploy/` 或 `simulate/`，都建议先清理旧构建缓存。

### 2.1 清理 B2 deploy build

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2
rm -rf build
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

### 2.2 清理 simulate build

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/simulate
rm -rf build
mkdir -p build
cd build
cmake ..
make -j$(nproc)
```

原因：

- `CMakeCache.txt` 里会记住旧源码路径
- 旧 `.o` 文件和新头文件/源码可能不匹配
- 即使能编译通过，也可能在运行时以 `free(): invalid pointer` 或 `corrupted size` 的形式崩掉

---

## 3. 运行前统一动态库来源

这一步非常重要。

如果系统里同时装了 ROS 2，最容易出现的问题是：

- `libddsc.so.0` 来自 ROS
- `libddscxx.so.0` 来自 `/usr/local/lib`

这种混用会导致：

- `dds_write... assertion failed`
- `free(): invalid pointer`
- `corrupted size vs. prev_size`

### 3.1 仿真运行前

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib
```

然后检查：

```bash
ldd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/simulate/build/unitree_mujoco | grep -E 'libddsc|libddscxx'
```

期望看到：

```text
libddsc.so.0 => /usr/local/lib/libddsc.so.0
libddscxx.so.0 => /usr/local/lib/libddscxx.so.0
```

### 3.2 B2 deploy 运行前

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
```

然后检查：

```bash
ldd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2/build/b2_ctrl | grep -E 'libddsc|libddscxx|libonnxruntime'
```

期望看到：

- `libddsc.so.0` 来自 `/usr/local/lib`
- `libddscxx.so.0` 来自 `/usr/local/lib`
- `libonnxruntime.so.1` 来自 `deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib`

不要看到 `/opt/ros/humble/...`

---

## 4. 运行顺序

### 4.1 仿真联调

终端 A：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/simulate/build
./unitree_mujoco
```

终端 B：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2/build
./b2_ctrl --network=lo
```

### 4.2 实机运行

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2/build
./b2_ctrl --network=<你的网卡名>
```

---

## 5. 更新后优先检查哪些文件

如果是从另一台机器拷回来的，优先确认下面这些是否来自同一套版本：

- `deploy/include/`
- `deploy/robots/b2/src/`
- `deploy/robots/b2/config/config.yaml`
- `deploy/robots/b2/config/policy/velocity/.../params/deploy.yaml`
- `deploy/robots/b2/config/policy/velocity/.../exported/policy.onnx`

尤其是：

- `policy.onnx`
- `deploy.yaml`

这两个如果不是同一套训练/导出链出来的，最容易出现：

- 进入 `Velocity` 时崩溃
- 运行时 observation/action 不匹配
- 模型加载成功但行为异常

---

## 6. 常见异常与对应处理

### 6.1 `bash: ./b2_ctrl: 权限不够`

可执行位丢了：

```bash
chmod +x /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/robots/b2/build/b2_ctrl
```

更推荐直接删掉 `build/` 后重新编译。

### 6.2 `fatal error: unitree/dds_wrapper/...: 没有那个文件或目录`

说明 `unitree_sdk2` 头文件没在当前 CMake include 路径中。

优先做法：

```bash
cd /home/vi/lxy/b2+z1/unitree_sdk2
mkdir -p build && cd build
cmake ..
sudo make install
```

### 6.3 `libddscxx.so.0 => not found`

说明运行时没找到 `/usr/local/lib`：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib
```

### 6.4 `libonnxruntime.so.1 => not found`

说明运行时没找到 ONNX Runtime：

```bash
unset LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/lib:/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib
```

### 6.5 `dds_write... assertion failed`

优先检查：

```bash
ldd simulate/build/unitree_mujoco | grep -E 'libddsc|libddscxx'
ldd deploy/robots/b2/build/b2_ctrl | grep -E 'libddsc|libddscxx'
```

如果 `libddsc` 和 `libddscxx` 不是来自同一路径，先统一到 `/usr/local/lib`。

### 6.6 `free(): invalid pointer` / `corrupted size vs. prev_size`

常见原因：

- 混用了 ROS 和 `/usr/local` 的 DDS 动态库
- 复制后 build 缓存没清
- `deploy.yaml` 和 `policy.onnx` 不是同一版本

处理顺序建议：

1. 先检查 `ldd`
2. 清理并重建 `build/`
3. 再确认 `policy.onnx` / `deploy.yaml` 是否来自同一套导出

---

## 7. 推荐操作习惯

如果以后仍然需要经常用 U 盘更新：

- 始终只拷源码和配置，不拷 `build/`
- 拷完之后固定执行“删 build -> cmake -> make”
- 运行前固定执行一次 `ldd | grep ddsc`
- 仿真和部署分别在干净终端里跑，不要混入 ROS 环境

这样基本可以把“拷贝后编译能过但运行崩溃”的概率降到最低。
