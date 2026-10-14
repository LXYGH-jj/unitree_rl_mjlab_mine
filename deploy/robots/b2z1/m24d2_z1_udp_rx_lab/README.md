# M2.4d2: 独立Z1 UDP接收证据实验台（仅localhost，非官方SDK协议）

**基线提交**：`7e23cf1`，分支 `feature/b2z1-interactive-deploy`。

## 已实现与限制

此模块使用真实 Linux IPv4 UDP `sendto` / `recvmsg` 在 `127.0.0.1` 上传输**专用测试协议**，检查完整数据报、源 IP/端口、CRC32、模拟包序号和接收时钟，并在同一次数据报解码后构造 `M2.4d1::RxEvidence` 与六关节 `Snapshot`。数据经 M2.4d1 `TransportSession` → M2.4c3 `JointCtrlSession`，最终**只能**进入内部 `RecordingSink`（内存容器），无法插入任何真实电机发送接口。

**重要：测试协议不是宇树Z1官方线协议**。其中 `Z1RX` magic、序号、CRC 和flags都是实验台自定义字段。真实 `RecvState` 公共头文件并不含此实验台序号，不能用本测试来推断真实 Z1 SDK 接收可靠性、报文新鲜性、抗重放能力或硬件允许控制。

官方预编译 `libZ1_SDK_x86_64.so` / `libZ1_SDK_aarch64.so` 中虽有 `UDPPort::recv()`、`recvfrom()` 符号，但本仓库提供的公开头文件没有提供经过审查的“完成接收 + 对端身份 + 对应解码关节状态”的原子回调。`M2.4d1` 的 `RxEvidence` 必须由真实经过审查的接收路径填写。**禁止**因调用 `sendRecv()`、读取 `lowstate`、或本地时钟更新而在真实程序中置 `confirmed_new_packet=true`。官方SDK的 `sendRecv()` **会发送UDP**；即便称为“观察”也不是只读。

本模块**独立编译**，不修改正式 `CMakeLists.txt`、FSM、`main.cpp`、M2.3、B2 SDK2 或宇树SDK；没有 `unitreeArm` 对象或物理发送端。与先前阶段不同，这次不把模拟收包器编入生产控制器，避免未来误用模拟事件放行真实输出。

## 测试协议（仅用于本机实验）

188字节，网络字节序：

- 字节0..3：magic `0x5A315258`（`Z1RX`）
- 字节4..5：版本 `1`
- 字节6..7：测试flags：bit0=connected；bit1=JOINTCTRL
- 字节8..15：测试序号 `uint64`，必须严格递增
- 字节16..183：六关节，每关节 `q,dq,tau` 各`float64`、温度`int16`、故障和连接码各`uint8`
- 字节184..187：CRC32（对前184字节标准CRC32的结果）

这个CRC只检测实验数据损坏，**不提供密码学身份验证**。IP/端口校验仅适合本机受控实验，不足以成为实机身份认证。不同时间收到相同位置值的两帧合法报文不能仅凭数值判定为重复；本实验台的显式序号用于验证重复模拟报文的拒绝流程，而不是宇树真实报文的能力。

## 开发电脑安装

```bash
cd ~/下载
unzip m24d2_z1_udp_rx_lab_overlay.zip
cd m24d2_z1_udp_rx_lab_overlay
REPO=/home/vi/lxy/unitree_ws/src/unitree_rl_mjlab
python3 tools/install_overlay.py "$REPO" --check
python3 tools/install_overlay.py "$REPO" --apply
bash run_offline_tests.sh "$REPO"
```

测试自动启动本机UDP收包测试，验证：正常数据、错误来源、短包/长包、CRC、序号重放、超温、电机故障、模式错误、超时及故障锁定，且成功通过M2.4d1安全门后，M2.4c3只向内存写入限速模拟命令。它无需MuJoCo/官方Z1程序或硬件网络。

可单独使用CMake：

```bash
cmake -S "$REPO/deploy/robots/b2z1/m24d2_z1_udp_rx_lab" -B /tmp/m24d2_z1_rx_build
cmake --build /tmp/m24d2_z1_rx_build -j2
ctest --test-dir /tmp/m24d2_z1_rx_build --output-on-failure
```

此前`b2z1_ctrl`的工程无需重新编译：本模块**不链接进真实控制程序**。

## 后续正式SDK接入门槛

1. 获取官方SDK `CtrlComponents::sendRecv`/`UDPPort::recv`实现源码，或官方提供的经审查接收事件回调；保证单一socket/线程接收所有权并确定状态同步机制。
2. 明确真实报文封包、序号（若存在）、校验、源端点；不能把实验台的自定义字段直接用于真实数据。
3. 实现具可复核证据的“完成recv → 从此包解码 → 时间戳 → 快照”原子路径；对重放风险缺少真实序号时要采取保守设计与独立硬件保护。
4. 接入真实JOINTCTRL发送端前，另行审核SDK后台发送线程所有权、控制停机、急停、供电/保险丝故障根因、机械限位和安装碰撞。

当前 `z1_jointctrl.enabled=true` 的硬件路线仍在B2 DDS初始化前拒绝执行；M2.4d2**不解除此门控**。

## Git 提交范围

```bash
git add -- deploy/robots/b2z1/m24d2_z1_udp_rx_lab/
git diff --cached --check
git --no-pager diff --cached --stat
```

本实验台不修改正式 CMake；不要提交`/tmp`测试构建、备份或旧LOWCMD实验。
