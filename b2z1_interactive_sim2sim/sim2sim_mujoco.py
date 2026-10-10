#!/usr/bin/env python3
"""Interactive B2Z1 low-level actor -> MuJoCo, with Z1 held in a fixed pose.

Pure MuJoCo Python runner. This does not publish DDS or send hardware commands.
The goal is policy/observation/action/physics validation, not full mobile manipulation.
"""
import argparse
import time
from pathlib import Path

import numpy as np
import torch
import mujoco

from lowlevel_core import WBCConfig, LowLevelWBC, RobotSample

# Interactive Actor policy/deployment order, NOT MuJoCo/native gym order:
LEG_PREFIXES = ('FR', 'FL', 'RR', 'RL')
LEG_JOINTS = tuple(f'{leg}_{part}_joint' for leg in LEG_PREFIXES for part in ('hip', 'thigh', 'calf'))
ARM_JOINTS = tuple(f'joint{i}' for i in range(1, 7))
JOINTS = LEG_JOINTS + ARM_JOINTS


def ids_for_model(model):
    qadr, dadr, a_ids = [], [], []
    for name in JOINTS:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise ValueError('Missing MuJoCo joint: ' + name)
        qadr.append(int(model.jnt_qposadr[jid]))
        dadr.append(int(model.jnt_dofadr[jid]))
        actuator_name = name[:-len('_joint')] if name.endswith('_joint') else name
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if aid < 0:
            raise ValueError('Missing torque actuator: ' + actuator_name)
        a_ids.append(int(aid))
    if len(set(a_ids)) != 18:
        raise ValueError('Actuator mapping has duplicate ids')
    return np.asarray(qadr), np.asarray(dadr), np.asarray(a_ids)


def quat_rot_inv_wxyz(quat, vec):
    w, x, y, z = np.asarray(quat, dtype=np.float64)
    v = np.asarray(vec, dtype=np.float64)
    xyz = np.array([x, y, z])
    t = -2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(-xyz, t)


def relative_tcp_goal(model, data):
    # Interactive uses an arm-base offset, yaw-rotated into the world frame.
    # For a no-moving-arm baseline, choose a constant goal that corresponds
    # to the currently held arm configuration at the initial posture.
    free_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base_joint')
    base_qadr = int(model.jnt_qposadr[free_id])
    base_p = data.qpos[base_qadr:base_qadr+3].copy()
    q = data.sensor('imu_quat').data.copy()
    # The initial yaw is 0 after setting the home keyframe.
    offset_world = np.array([0.2, 0.0, 0.09])
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'tcp')
    if tcp_id < 0:
        raise ValueError('Missing MuJoCo body tcp')
    goal_world = data.xpos[tcp_id].copy()
    return quat_rot_inv_wxyz(q, goal_world - base_p - offset_world).astype(np.float32)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--xml', required=True, help='path to scene_b2z1.xml')
    p.add_argument('--policy', required=True, help='history-encoder TorchScript JIT policy')
    p.add_argument('--config', default=str(Path(__file__).with_name('config_lowlevel.yaml')))
    p.add_argument('--vx', type=float, default=0.2)
    p.add_argument('--vy', type=float, default=0.0)
    p.add_argument('--wz', type=float, default=0.0)
    p.add_argument('--seconds', type=float, default=8.0)
    p.add_argument('--warmup', type=float, default=1.0)
    p.add_argument('--physics-dt', type=float, default=0.005)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--realtime', action='store_true')
    args = p.parse_args()
    if args.seconds <= 0 or args.warmup < 0 or args.physics_dt <= 0:
        p.error('seconds/physics-dt must be >0 and warmup >=0')
    if max(abs(args.vx), abs(args.vy), abs(args.wz)) > 0.6:
        p.error('This first simulation test limits abs(command) <= 0.6')

    cfg = WBCConfig.load(args.config)
    policy = torch.jit.load(str(Path(args.policy).expanduser()), map_location='cpu').eval()
    wbc = LowLevelWBC(cfg, policy)
    model = mujoco.MjModel.from_xml_path(str(Path(args.xml).expanduser().resolve()))
    model.opt.timestep = args.physics_dt
    nsubsteps = round(cfg.control_dt / model.opt.timestep)
    if nsubsteps < 1 or abs(nsubsteps * model.opt.timestep - cfg.control_dt) > 1e-7:
        raise ValueError('control_dt must be an integer multiple of physics-dt')
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if home >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home)
    else:
        mujoco.mj_resetData(model, data)
    qadr, dadr, actuator_ids = ids_for_model(model)
    data.qpos[qadr] = cfg.default_joint_pos   # Starts legs and Z1 at Interactive training posture
    # Keep original keyframe root position, but raise if underneath the floor.
    mujoco.mj_forward(model, data)
    ee_local = relative_tcp_goal(model, data)
    print('Loaded:', args.xml)
    print('Model nq,nv,nu:', model.nq, model.nv, model.nu)
    print('Policy joints:', JOINTS)
    print('Fixed EE policy target [arm-base/body frame]:', ee_local)
    print('Control_dt=%.3f, Physics_dt=%.3f, substeps=%d' % (cfg.control_dt, model.opt.timestep, nsubsteps))
    print('Mode: direct Python MuJoCo, no ROS/DDS/no real motor output')

    # These are simulator torque-control gains, NOT validated hardware gains.
    # The MuJoCo XML uses <motor> actuators, not <position> actuators.
    kp = np.asarray([360.0]*12 + [60.0]*6, dtype=np.float64)
    kd = np.asarray([5.0]*12 + [4.0]*6, dtype=np.float64)
    ctrl_low = model.actuator_ctrlrange[actuator_ids, 0].copy()
    ctrl_high = model.actuator_ctrlrange[actuator_ids, 1].copy()
    start_x = float(data.qpos[0])
    viewer = None
    if not args.headless:
        from mujoco import viewer as mj_viewer
        viewer = mujoco.viewer.launch_passive(model, data)

    last_target = cfg.default_joint_pos.astype(np.float64).copy()
    nsteps = int(round(args.seconds / cfg.control_dt))
    try:
        for k in range(nsteps):
            tick = time.perf_counter()
            t = k * cfg.control_dt
            cmd = np.array([args.vx, args.vy, args.wz], dtype=np.float32)
            if t < args.warmup:
                cmd[:] = 0.0
            sample = RobotSample(
                joint_q=data.qpos[qadr].astype(np.float32).copy(),
                joint_dq=data.qvel[dadr].astype(np.float32).copy(),
                imu_quat_wxyz=np.asarray(data.sensor('imu_quat').data, dtype=np.float32).copy(),
                imu_gyro_xyz=np.asarray(data.sensor('imu_gyro').data, dtype=np.float32).copy(),
                ee_local_xyz=ee_local.copy(),
            )
            with torch.no_grad():
                targets, actions, obs = wbc.act(sample, cmd)
            if obs.size != 799 or actions.size != 18:
                raise RuntimeError('Unexpected policy observation/action size')
            last_target[:12] = targets[:12]
            last_target[12:] = cfg.default_joint_pos[12:]  # Z1 fixed, not using RL arm output
            for _ in range(nsubsteps):
                pos = data.qpos[qadr]
                vel = data.qvel[dadr]
                torque = kp * (last_target - pos) - kd * vel
                if not np.isfinite(torque).all():
                    raise RuntimeError('Nonfinite motor torque; abort')
                data.ctrl[actuator_ids] = np.clip(torque, ctrl_low, ctrl_high)
                mujoco.mj_step(model, data)
            if k % 25 == 0 or k == nsteps - 1:
                print('t=%.2fs cmd_vx=%.2f root_x=%.3f root_z=%.3f act_leg_max=%.2f' % (
                    t, cmd[0], data.qpos[0], data.qpos[2], np.max(np.abs(actions[:12]))), flush=True)
            if viewer is not None:
                if not viewer.is_running():
                    break
                viewer.sync()
            if args.realtime:
                time.sleep(max(0.0, cfg.control_dt - (time.perf_counter() - tick)))
        print('World-X displacement: %.3f m' % (float(data.qpos[0]) - start_x))
    finally:
        if viewer is not None:
            viewer.close()

if __name__ == '__main__':
    main()
