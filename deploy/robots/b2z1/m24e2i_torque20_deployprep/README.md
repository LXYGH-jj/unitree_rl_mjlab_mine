# M2.4e2i — 20 N·m 前馈检查 + 实机部署迁移准备

## 状态与范围

M2.4e2h 已通过 13 项 mock + SDK 真分支链接；M2.4e2g 原生 B2 FSM 编译通过。当前包只将 M2.4e2d SDK 桥接所引用的 `m24e2h_gravity_guard.h` 中的前馈数值 **绝对值**门限由 `50.0` 调到 `20.0` N·m，**超过阈值抛异常，不会截断命令继续发送**。单次修改可备份/回滚。没有改变电机的 PD 增益、总力矩、电机硬件限流或其他控制器路径。

> 20 N·m 不是官方确认的 Z1 所有关节连续/峰值力矩值。宇树官方 Z1 产品页给出了关节驱动器的 33 N·m 最大扭矩（没有逐关节连续额定表）；`z1_controller/include/model/unitree_gripper.h` 的 MAX_TORQUE=20.0 属于夹爪，**不能作为6关节的统一力矩规格**。

参考： https://www.unitree.com/cn/z1/ ; https://github.com/unitreerobotics/z1_controller/blob/master/include/model/unitree_gripper.h

## 1. 在开发电脑打补丁

```bash
cd ~/下载
unzip m24e2i_torque20_deployprep.zip
cd m24e2i_torque20_deployprep
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash "$REPO/deploy/robots/b2z1/m24e2i_torque20_deployprep/run_offline_tests.sh" "$REPO"
bash "$REPO/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/run_sdk_linkcheck.sh" "$REPO"
```

自动备份到：`deploy/robots/b2z1/m24e2i_torque20_deployprep/backups/m24e2h_gravity_guard.h.before20`。

## 2. 先决定控制进程运行在哪一台机器

**A. 当前开发PC直接控制B2+Z1（首选短期台架方案）**：`/home/vi/...` 中源码、B2 SDK2、x86_64 Z1 SDK 已就位；通过 PC 有线口与 B2/Z1 网络互通，B2 DDS+Z1 SDK Worker+主FSM都运行在 PC 上，**无需上传到B2电脑或机械臂**，只需要本机重新编译目标。注意 Z1 SDK 的 UDP 配置和 `z1_ctrl` 的启动方式，一台Z1只允许一个主动控制者。

**B. 换到另一台 Linux 控制PC**：先在目标机准备目录与依赖，rsync `deploy/` 源码、模型、第三方资源，**目标机重新编译**。`rsync` 脚本默认只 dry-run，不会执行 SSH 远端命令，不会传输历史 build 产物。

**C. 板载 Jetson/aarch64**：必须有 `libZ1_SDK_aarch64.so`、对应的 ONNX Runtime aarch64、匹配的 SDK2、Eigen、C++ ABI / glibc。不能直接拷贝 PC 上 `x86_64` 的 `b2z1_ctrl` 或 `libZ1_SDK_x86_64.so`。首次台架建议尽量使用已有 x86_64 PC，减少交叉编译风险。

## 3. 异机迁移（需替换实际目标账号/IP）

### 开发机：确定 target 目录，先 dry-run，再传

```bash
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
REMOTE=user@192.168.123.X      # 根据实际控制机替换；不是Z1的设备IP
DEST=/home/user/unitree_rl_mjlab # 根据目标机用户主目录替换
ssh "$REMOTE" "mkdir -p '$DEST'"  # 需先在目标机备好目录
bash "$REPO/deploy/robots/b2z1/m24e2i_torque20_deployprep/tools/transfer_deploy.sh" "$REPO" "$REMOTE" "$DEST" --check
bash "$REPO/deploy/robots/b2z1/m24e2i_torque20_deployprep/tools/transfer_deploy.sh" "$REPO" "$REMOTE" "$DEST" --apply
```

脚本不会使用 `--delete`，不会拷贝 `build/`；可能传输较大的 ONNX Runtime 及模型文件。如果需要完全复现整个 Git 工作树，另用版本控制方式同步当前特性分支和未提交改动，不只传输该 `deploy/` 目录。

### 目标控制机：本地检查架构、SDK库、动态依赖，重新编译

```bash
DEST=/home/user/unitree_rl_mjlab  # 改成实际目标路径
python3 "$DEST/deploy/robots/b2z1/m24e2i_torque20_deployprep/tools/preflight_target.py" "$DEST"
cmake -S "$DEST/deploy/robots/b2z1/m24e2d_sdk_worker_bridge" \
      -B "$DEST/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/build" \
      -DM24E2D_ENABLE_HARDWARE_SDK=ON -DCMAKE_BUILD_TYPE=Release
cmake --build "$DEST/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/build" -j4
cmake -S "$DEST/deploy/robots/b2z1" -B "$DEST/deploy/robots/b2z1/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$DEST/deploy/robots/b2z1/build" -j4 --target b2z1_ctrl
ldd "$DEST/deploy/robots/b2z1/m24e2d_sdk_worker_bridge/build/m24e2d_z1_worker" | grep 'not found' || true
ldd "$DEST/deploy/robots/b2z1/build/b2z1_ctrl" | grep 'not found' || true
```

`preflight_target.py` 不控制机器人；`cmake --build` 仅构建，不启动。库缺失时查依赖/链接配置，不要临时使用未验证的混合版 SDK 动态库。目标机 `ldd` 无 `not found` 只是依赖满足，**不是电机安全验收**。

## 4. 上电和真实操作前的门禁

- 仅在机械臂固定支撑、急停、电源、碰撞空间和旁站人员全部就位后，考虑 SDK 模式测试。
- `z1_ctrl k` 已验证的是官方控制路径。自研 LOWCMD 仍需独立验证重力模型/真实负载、PD、实际通讯新包、500Hz收发与停止行为。
- SDK Worker 启动时可能切换模式并驱动电机，**不能当作纯反馈查看器**。不能和 `z1_ctrl k` 同时运行。
- 必须先解决 Z1 Worker SIGTERM/断线以后有物理实证的安全停机方式，再准许实际低层连续轨迹控制。
- `M2.4e2g` 的 B2 真实输出默认关闭、Z1目标强制锁定，**不允许仅为测试连接而打开任何输出门禁**；否则 Passive/FixStand 路径同样可能有电机控制效果。
- 控制权建议：PC键盘→原生 B2 FSM (12 关节 DDS)；Z1目标经同机Unix Socket→单独SDK Worker；一个Z1只有一个 SDK LOWCMD owner。

## 5. 校验本机运行时资源

```bash
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 "$REPO/deploy/robots/b2z1/m24e2i_torque20_deployprep/tools/preflight_target.py" "$REPO"
file "$REPO/deploy/robots/b2z1/build/b2z1_ctrl"
```

验证通过后再讨论真实台架动作。当前包只完善软件检查与迁移准备，不允许据此直接运行 `--backend sdk`。
