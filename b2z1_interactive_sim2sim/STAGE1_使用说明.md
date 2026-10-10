# Interactive → MuJoCo 第一阶段：可比性实验（不修改现有程序）

第一组实验采用双边“固定Z1默认关节目标”测试，暂不做机械臂动态IK末端跟踪。用同一 TorchScript Actor `[799] → [18]`。

**涉及的变化：**
- 新建 MuJoCo 机器人和场景XML（Z1安装位置修改成训练时 `[0.2,0,0.1]`）。
- MuJoCo初始高度设0.6m、Z1控制增益5/0.5、取消此前添加的10个周期腿动作保持、取消额外的腿部目标角度截断（仅限纯仿真测试）。
- Gym临时跳过原始IK，固定Z1至原训练默认角度，其他训练文件不修改。
- MuJoCo使用原Gym teleop模式的目标坐标近似重建，实时依据机身姿态更新。
- **质量、惯量、碰撞、足地摩擦暂未对齐。** 本实验属于控制与观测迁移的初步基线，不是完全物理参数一致的证明。

## 复制三个脚本
把三个 `.py` 放到 `/home/vi/lxy/b2z1_ws/B2Z1_Interactive_MuJoCo_Sim2Sim/b2z1_interactive_sim2sim/`。

## 1. 生成独立MuJoCo场景（只执行一次）

```bash
cd /home/vi/lxy/b2z1_ws/B2Z1_Interactive_MuJoCo_Sim2Sim/b2z1_interactive_sim2sim
python prepare_stage1_model.py --xml-dir /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/src/assets/robots/unitree_b2z1/xmls
```

若已生成同名对照文件，脚本不会覆盖（主动防止丢失人工修改）。

## 2. MuJoCo 6秒

```bash
env LD_LIBRARY_PATH="$CONDA_PREFIX/lib" TORCH_EXTENSIONS_DIR="$CONDA_PREFIX/torch_extensions" MAX_JOBS=2 PYTHONNOUSERSITE=1 \
python sim2sim_interactive_stage1.py \
 --xml /home/vi/lxy/unitree_ws/src/unitree_rl_mjlab/src/assets/robots/unitree_b2z1/xmls/scene_b2z1_interactive_stage1.xml \
 --policy ./b2z1_low_policy_jit.pt --vx 0.30 --seconds 6 --warmup 1 \
 --csv ./stage1_mujoco.csv | tee stage1_mujoco.log
```

## 3. 对应Isaac Gym 6秒

```bash
env LD_LIBRARY_PATH="$CONDA_PREFIX/lib" TORCH_EXTENSIONS_DIR="$CONDA_PREFIX/torch_extensions" MAX_JOBS=2 PYTHONNOUSERSITE=1 \
python gym_interactive_stage1_static_arm.py \
 --interactive-root /home/vi/lxy/b2z1_ws/Interactive-Navigation-for-legged-manipulator/low_level_WBC \
 --policy ./b2z1_low_policy_jit.pt --vx 0.30 --seconds 6 --warmup 1 \
 --csv ./stage1_gym.csv | tee stage1_gym.log
```

## 4. 解读结果

重点比较两个输出的 `avg_body_vx`, `avg_body_vy`, `avg_root_z`, `final_yaw`；命令一致且arm都使用相同静态关节目标。还应比较观测中末端目标的前三个元素，确定Gym的球面坐标约定与本脚本完全一致，若不一致，不要继续推断动力学差异。

**重要限制：** 第一阶段还需要验证799维逐维观测和物理参数对齐；此处并未实现完整obs转储。测试与真实机器人无关。
