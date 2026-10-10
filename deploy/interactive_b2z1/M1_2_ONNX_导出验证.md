# M1.2：Interactive TorchScript → ONNX（只进行离线验证）

## 使用约束

- 固定源模型：`b2z1_interactive_sim2sim/b2z1_low_policy_jit.pt`。脚本不会修改源模型。
- 固定输入输出协议：`obs` float32 `[1,799]` → `actions` float32 `[1,18]`。
- ONNX opset 默认 13，保持固定 batch=1。与当前 `deploy/include/isaaclab/algorithms/algorithms.h` 的常规 OrtRunner 兼容性要在后续 C++ 测试单独验证。
- *绝对不要*将候选 ONNX 覆盖已有 `deploy/robots/b2z1/config/policy/.../exported/policy.onnx`。
- 此阶段不运行 SDK2、Z1 SDK、ROS、DDS、FSM，也不产生任何电机控制命令。

## 1. 预检环境

从仓库根目录：

```bash
conda activate b2z1_wbc
python - <<'PY'
import sys, torch
print('Python:', sys.version)
print('Torch:', torch.__version__)
for name in ('onnx', 'onnxruntime'):
    try:
        m = __import__(name)
        print(name, m.__version__)
    except Exception as e:
        print(name, 'NOT READY:', repr(e))
PY
```

**若缺 `onnx` 或 `onnxruntime`，不要盲目升级 Torch、NumPy 或 Isaac Gym**。先保存环境清单；可以在兼容环境中安装所需运行库，并保留旧训练环境不动。TorchScript导出依赖PyTorch的ONNX exporter；onnxruntime仅用于验证。

## 2. 导出与数值检查

确保上一阶段的 `interactive_obs_fixture` 已成功编译，然后运行：

```bash
cd /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python deploy/interactive_b2z1/tests/export_verify_interactive_onnx.py \
  --policy b2z1_interactive_sim2sim/b2z1_low_policy_jit.pt \
  --output deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx \
  --fixture deploy/interactive_b2z1/build/interactive_obs_fixture
```

脚本执行的检查包括：冻结源权重 SHA256、源 TorchScript 输出形状和有限数值、ONNX 图结构与静态输入输出shape、ONNX Runtime与 TorchScript 对零输入/两组随机输入/5个连续的C++观测帧的动作数值一致性。

默认容差为`atol=1e-4`、`rtol=1e-4`。只有全部通过才会生成最终 ONNX；校验失败不会覆盖已有输出。

成功后检查输出：

```bash
sha256sum b2z1_interactive_sim2sim/b2z1_low_policy_jit.pt \
  deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx
ls -lh deploy/interactive_b2z1/build/b2z1_interactive_policy.onnx
```

ONNX候选保存在 `.gitignore` 忽略的本地`build/`目录，不会被`git add deploy/interactive_b2z1/`误加入Git。后续验证完毕再决定统一模型存放位置与版本管理方式。

## 3. 验收与风险

- 终端应出现 `M1.2 ONNX PARITY: PASS`，否则不要接入现有FSM。
- 如果TorchScript actor中包含当前Torch版本的ONNX导出不支持的算子，脚本会停止且不修改源权重。需要根据完整错误栈选择导出方案，不能直接忽略。
- ONNX与TorchScript推理结果一致并不意味着关节顺序、IMU、实际Z1几何、反馈时间戳或安全保护已验证。下一步应独立验证C++ ONNX Runtime输出，再考虑在原控制器中只读接入。
