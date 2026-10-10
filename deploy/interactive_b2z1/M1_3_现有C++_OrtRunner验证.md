# M1.3（第一步）：现有C++ OrtRunner离线一致性测试

本测试使用`deploy/include/isaaclab/algorithms/algorithms.h`中**现有的`isaaclab::OrtRunner`类**，而不是另外实现一个推理器。仅对5帧离线799维观测执行推理，输出18维原始动作。不会访问DDS、SDK2、Z1 SDK或机器人电机。

前置条件：
- 当前分支：`feature/b2z1-interactive-deploy`。
- M1.1生成的`deploy/interactive_b2z1/build/interactive_obs_fixture`。
- M1.2生成的`deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx`。
- 原始PyTorch 1.10生成的`deploy/interactive_b2z1/build/actions_torch110.npy`。
- C++ ONNX Runtime 1.22动态库：`deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib/libonnxruntime.so.1.22.0`。

## 1. 检查依赖

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
ORT_ROOT="$PWD/deploy/thirdparty/onnxruntime-linux-x64-1.22.0"
ls -l "$ORT_ROOT/include/onnxruntime_cxx_api.h" \
      "$ORT_ROOT/lib/libonnxruntime.so.1.22.0"

# 核实IR/opset；过高的IR版本可能不兼容C++ ORT 1.22。
/home/vi/lxy/unitree_ws/src/astro_parkour/.venv/bin/python - <<'PY'
import onnx
p = 'deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx'
m = onnx.load(p, load_external_data=False)
print('IR version:', m.ir_version)
print('Opset:', [(x.domain, x.version) for x in m.opset_import])
PY
```

## 2. 仅编译测试工具（不重编完整B2Z1控制器）

```bash
mkdir -p deploy/interactive_b2z1/build

g++ -std=c++17 -O2 -Wall -Wextra \
  -I"$PWD/deploy/include" \
  -I"$ORT_ROOT/include" \
  deploy/interactive_b2z1/tests/test_cpp_ort_runner.cpp \
  "$ORT_ROOT/lib/libonnxruntime.so.1.22.0" \
  -Wl,-rpath,"$ORT_ROOT/lib" \
  -pthread \
  -o deploy/interactive_b2z1/build/test_cpp_ort_runner
```

## 3. 与原PyTorch 1.10动作逐项对比

```bash
/home/vi/miniconda3/envs/b2z1_wbc/bin/python \
  deploy/interactive_b2z1/tests/test_cpp_ort_parity.py \
  --binary deploy/interactive_b2z1/build/test_cpp_ort_runner \
  --onnx deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx \
  --fixture deploy/interactive_b2z1/build/interactive_obs_fixture \
  --reference deploy/interactive_b2z1/build/actions_torch110.npy
```

预期：`M1.3 C++ OrtRunner PARITY: PASS`。若出现`Unsupported model IR version`，先记录报错，不要直接手动修改ONNX IR字段或替换部署C++ Runtime；需要检查实际算子和IR兼容性。

此测试通过只表示**已有C++推理类与旧TorchScript输出在5帧数据上对齐**；还未接入`State_RLBase`，也未验证实机动作、延迟、频率、SDK2映射和Z1控制安全。
