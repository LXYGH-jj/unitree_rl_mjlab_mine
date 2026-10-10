#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""B2+Z1 simulation-only: Interactive leg RL + six-joint Z1 position IK.

Keyboard commands are isolated from MuJoCo Viewer: a separate tkinter control
panel (or terminal raw-key mode) owns all robot shortcuts. MuJoCo Viewer is
launched WITHOUT key_callback.

The unactuated ``jointGripper`` is NOT a reliable IK end effector. This script
tracks the rigid wrist-equivalent tool point on link06 (default +0.15m X).
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import select
import sys
import threading
import time
from pathlib import Path

import mujoco
import numpy as np
import torch
from mujoco import viewer as mj_viewer

LEG_PREFIXES = ("FR", "FL", "RR", "RL")
LEG_NAMES = tuple(f"{side}_{part}_joint" for side in LEG_PREFIXES for part in ("hip", "thigh", "calf"))
ARM_NAMES = tuple(f"joint{i}" for i in range(1, 7))
JOINT_NAMES = LEG_NAMES + ARM_NAMES


def joint_indices(model):
    qa, va, aid, jid = [], [], [], []
    for name in JOINT_NAMES:
        j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        actuator_name = name[:-6] if name.endswith("_joint") else name
        a = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if min(j, a) < 0:
            raise RuntimeError(f"Cannot find joint/actuator {name} / {actuator_name}")
        jid.append(j); qa.append(model.jnt_qposadr[j]); va.append(model.jnt_dofadr[j]); aid.append(a)
    return (np.array(qa, dtype=int), np.array(va, dtype=int),
            np.array(aid, dtype=int), np.array(jid, dtype=int))


def body_rot(data, bid):
    return np.array(data.xmat[bid], dtype=float).reshape(3, 3)


def yaw_from_quat(q):
    w, x, y, z = map(float, q)
    return math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))


def yaw_rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


class RigidTCP:
    """Tool point fixed to link06, rather than uncontrolled gripperMover TCP."""
    def __init__(self, model, wrist_body, offset_x=0.15):
        self.model = model
        self.wrist_body = wrist_body
        self.offset = np.array([offset_x, 0., 0.], dtype=float)

    def position(self, data):
        return (np.asarray(data.xpos[self.wrist_body], dtype=float)
                + body_rot(data, self.wrist_body) @ self.offset)

    def position_jacobian(self, data, arm_vadr):
        jacp = np.zeros((3, self.model.nv), dtype=float)
        jacr = np.zeros((3, self.model.nv), dtype=float)
        p = self.position(data)
        mujoco.mj_jac(self.model, data, jacp, jacr, p, self.wrist_body)
        return jacp[:, arm_vadr].copy()


class Goal:
    def __init__(self, data, mount_body, tcp):
        self.mount = mount_body
        self.tcp = tcp
        self.mode = 'body'
        self.local = np.zeros(3, dtype=float)
        self.world = np.zeros(3, dtype=float)
        self.capture(data)

    def mount_pose(self, data):
        return np.asarray(data.xpos[self.mount], dtype=float), body_rot(data, self.mount)

    def capture(self, data):
        p, R = self.mount_pose(data)
        self.world = self.tcp.position(data).copy()
        self.local = R.T @ (self.world - p)

    def target(self, data):
        if self.mode == 'world':
            return self.world.copy()
        p, R = self.mount_pose(data)
        return p + R @ self.local

    def toggle(self, data):
        world = self.target(data)
        p, R = self.mount_pose(data)
        self.mode = 'world' if self.mode == 'body' else 'body'
        self.world = world
        self.local = R.T @ (world - p)
        return self.mode

    def move(self, axis, delta):
        if self.mode == 'world':
            self.world[axis] += delta
        else:
            self.local[axis] += delta


def ik_target(model, data, tcp, arm_qa, arm_va, arm_jid, target_world,
              damping, gain, max_joint_step, max_task_error):
    p = tcp.position(data)
    e_full = np.asarray(target_world) - p
    dist = float(np.linalg.norm(e_full))
    e = e_full * min(1., max_task_error / max(dist, 1.e-12))
    J = tcp.position_jacobian(data, arm_va)
    singular = np.linalg.svd(J, compute_uv=False)
    # Damped least-squares position IK; no base DOF is actuated by this IK.
    qstep = gain * J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(3), e)
    qstep = np.clip(qstep, -max_joint_step, max_joint_step)
    q_now = np.asarray(data.qpos[arm_qa], dtype=float)
    target = q_now + qstep
    joint_limited = model.jnt_limited[arm_jid].astype(bool)
    lo = model.jnt_range[arm_jid, 0]
    hi = model.jnt_range[arm_jid, 1]
    target = np.where(joint_limited, np.clip(target, lo, hi), target)
    if not np.isfinite(target).all():
        raise RuntimeError('IK produced NaN/Inf')
    return target, dist, float(np.linalg.norm(qstep)), singular


class Commands:
    def __init__(self):
        self.events = queue.Queue()
        self.status = queue.Queue(maxsize=1)
        self.terminate = threading.Event()

    def push(self, key):
        self.events.put(key.upper())

    def drain(self):
        out = []
        try:
            while True:
                out.append(self.events.get_nowait())
        except queue.Empty:
            return out

    def set_status(self, text):
        try:
            self.status.get_nowait()
        except queue.Empty:
            pass
        self.status.put_nowait(text)


def panel_worker(io):
    """Run all tkinter interactions exclusively from this UI thread (Linux)."""
    try:
        import tkinter as tk
    except ImportError as e:
        print('[INPUT ERROR] tkinter missing:', e, '\nUse --input-mode terminal', flush=True)
        io.terminate.set(); return
    try:
        root = tk.Tk()
    except Exception as e:
        print('[INPUT ERROR] Could not open control panel:', e, '\nUse --input-mode terminal', flush=True)
        io.terminate.set(); return
    root.title('B2+Z1 独立控制台 | 点击此窗口后按键')
    root.geometry('580x530')
    root.resizable(False, False)
    tk.Label(root, text='键盘只在此控制台生效；MuJoCo Viewer 不接管控制指令',
             font=('Arial', 12, 'bold'), wraplength=550).pack(pady=(12, 6))
    tk.Label(root, text='B2: W/S 前后, A/D 横移, Q/E 转向, 空格停止\n'
                        'Z1: I/K 前后, J/L 左右, U/O 上下, M 切换坐标系, T 重置目标\n'
                        'P 暂停; X 停止速度指令并重置末端目标 (仅仿真)',
             justify='left').pack(pady=4)
    status = tk.StringVar(value='等待MuJoCo状态...')
    tk.Label(root, textvariable=status, font=('Courier', 10),
             justify='left', anchor='w', bg='#eeeeee', wraplength=540,
             width=70, height=6).pack(padx=12, pady=8)
    sections = [
        ('B2 基座控制', [('W 前进', 'W'), ('S 后退', 'S'), ('A 左移', 'A'),
                     ('D 右移', 'D'), ('Q 左转', 'Q'), ('E 右转', 'E'), ('空格停止', ' ')]),
        ('Z1 末端增量', [('I +X', 'I'), ('K -X', 'K'), ('J +Y', 'J'),
                       ('L -Y', 'L'), ('U +Z', 'U'), ('O -Z', 'O')]),
        ('其他', [('M 切换目标坐标系', 'M'), ('T 目标=当前', 'T'),
                ('P 暂停', 'P'), ('X 复位指令', 'X')]),
    ]
    for label, binds in sections:
        grp = tk.LabelFrame(root, text=label, padx=8, pady=5)
        grp.pack(fill='x', padx=12, pady=3)
        for i, (caption, key) in enumerate(binds):
            b = tk.Button(grp, text=caption, command=lambda k=key: io.push(k), width=18)
            b.grid(row=i//4, column=i%4, padx=3, pady=3)
    def on_key(event):
        key = event.keysym.lower()
        mapping = {'space': ' ', 'escape': 'X'}
        mapped = mapping.get(key, key.upper())
        if mapped in set('WSADQEIKJLUOMTPX') | {' '}:
            io.push(mapped)
            return 'break'
    root.bind_all('<KeyPress>', on_key)
    def refresh():
        try:
            while True:
                status.set(io.status.get_nowait())
        except queue.Empty:
            pass
        if not io.terminate.is_set():
            root.after(100, refresh)
        else:
            root.destroy()
    def close():
        io.terminate.set()
        root.destroy()
    root.protocol('WM_DELETE_WINDOW', close)
    root.after(100, refresh)
    root.mainloop()


def terminal_worker(io):
    """Read raw keyboard from this terminal; Viewer never sees those keys."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    if not sys.stdin.isatty():
        print('[INPUT ERROR] --input-mode terminal needs an interactive TTY', flush=True)
        io.terminate.set(); return
    original = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        print('\n[TERMINAL] Keep keyboard focus in this terminal, NOT the MuJoCo window.\n'
              'W/S/A/D/Q/E base | I/K/J/L/U/O arm | Space stop | M toggle | T recapture | P pause | X reset', flush=True)
        while not io.terminate.is_set():
            available, _, _ = select.select([fd], [], [], .1)
            if not available:
                continue
            key = os.read(fd, 1).decode('utf-8', errors='ignore')
            if key and key.upper() in 'WSADQEIKJLUOMTPX ':
                io.push(key)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, original)


def viewer_markers(viewer, goal_xyz, tcp_xyz):
    """Orange = goal, cyan = rigid TCP. user_scn separate from model scene."""
    try:
        with viewer.lock():
            scn = viewer.user_scn
            count = min(2, scn.maxgeom)
            scn.ngeom = count
            for i, (point, rgba) in enumerate((
                (goal_xyz, [1., .42, .03, .95]),
                (tcp_xyz, [.05, .95, 1., .95]),
            )):
                if i >= count:
                    break
                mujoco.mjv_initGeom(scn.geoms[i], mujoco.mjtGeom.mjGEOM_SPHERE,
                                   np.array([.025 if i == 0 else .018, 0., 0.]),
                                   np.asarray(point, dtype=float), np.eye(3).reshape(-1),
                                   np.array(rgba, dtype=float))
    except (AttributeError, TypeError, ValueError) as exc:
        print('[VIEW WARNING]', exc, flush=True)


def main():
    ap = argparse.ArgumentParser(description='B2+Z1 MuJoCo RL+IK (keyboard isolated from viewer)')
    ap.add_argument('--xml', required=True)
    ap.add_argument('--policy', required=True)
    ap.add_argument('--wbc-dir', required=True)
    ap.add_argument('--config', default='')
    ap.add_argument('--input-mode', choices=['panel', 'terminal'], default='panel')
    ap.add_argument('--physics-dt', type=float, default=.005)
    ap.add_argument('--initial-z', type=float, default=.6)
    ap.add_argument('--arm-kp', type=float, default=90.)
    ap.add_argument('--arm-kd', type=float, default=6.)
    ap.add_argument('--arm-gravity-comp', choices=['on', 'off'], default='on')
    ap.add_argument('--arm-step', type=float, default=.015)
    ap.add_argument('--tcp-offset-x', type=float, default=.15)
    ap.add_argument('--ik-damping', type=float, default=.07)
    ap.add_argument('--ik-gain', type=float, default=.65)
    ap.add_argument('--ik-max-joint-step', type=float, default=.035)
    ap.add_argument('--ik-max-error', type=float, default=.20)
    ap.add_argument('--max-steps', type=int, default=0)
    ap.add_argument('--csv', default='')
    args = ap.parse_args()
    if min(args.arm_kp, args.arm_kd, args.arm_step,
           args.ik_damping, args.ik_gain, args.ik_max_joint_step, args.ik_max_error) <= 0:
        ap.error('IK / servo tuning must be positive')

    sys.path.insert(0, str(Path(args.wbc_dir).expanduser().resolve()))
    from lowlevel_core import LowLevelWBC, RobotSample, WBCConfig
    config_path = Path(args.config or Path(args.wbc_dir) / 'config_lowlevel.yaml')
    cfg = WBCConfig.load(config_path)
    cfg.startup_hold_cycles = 0
    # Match original Gym RL leg target computation (SIM ONLY).
    cfg.leg_min[:] = -1.e6
    cfg.leg_max[:] = 1.e6
    net = torch.jit.load(str(Path(args.policy).expanduser().resolve()), map_location='cpu').eval()
    wbc = LowLevelWBC(cfg, net)

    model = mujoco.MjModel.from_xml_path(str(Path(args.xml).expanduser().resolve()))
    model.opt.timestep = args.physics_dt
    nsub = round(cfg.control_dt / model.opt.timestep)
    if nsub < 1 or abs(nsub * model.opt.timestep - cfg.control_dt) > 1e-8:
        ap.error('control_dt must be divisible by physics-dt')
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'home')
    if home >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home)
    else:
        mujoco.mj_resetData(model, data)
    qa, va, aa, ji = joint_indices(model)
    root_j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'floating_base_joint')
    mount = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link00')
    wrist = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link06')
    if min(root_j, mount, wrist) < 0:
        raise RuntimeError('Missing floating_base_joint, link00 or link06')
    root_qa = int(model.jnt_qposadr[root_j])
    base_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
    tcp = RigidTCP(model, wrist, args.tcp_offset_x)
    data.qpos[root_qa + 2] = args.initial_z
    data.qpos[qa] = cfg.default_joint_pos
    data.qvel[:] = 0.
    mujoco.mj_forward(model, data)

    goal = Goal(data, mount, tcp)
    cmd = np.zeros(3, dtype=np.float32)
    kp = np.r_[np.full(12, 360.), np.full(6, args.arm_kp)]
    kd = np.r_[np.full(12, 5.), np.full(6, args.arm_kd)]
    lo = model.actuator_ctrlrange[aa, 0].copy()
    hi = model.actuator_ctrlrange[aa, 1].copy()
    arm_qa, arm_va, arm_ji = qa[12:], va[12:], ji[12:]
    arm_nominal_base_offset = np.array([.20, 0., .09], dtype=float)
    io = Commands()
    paused, tick = False, 0
    rows = []
    start_io = panel_worker if args.input_mode == 'panel' else terminal_worker
    threading.Thread(target=start_io, args=(io,), daemon=True).start()
    print('B2+Z1 interaction v2 (SIMULATION ONLY).')
    print('Keyboard focus: separate control panel / terminal, NOT MuJoCo Viewer.')
    print('IK: link06 rigid tool point (local x={:.3f}m), avoiding passive jointGripper.'.format(args.tcp_offset_x))
    print('Arm PD kp/kd={:.1f}/{:.1f}, gravity compensation={}'.format(args.arm_kp, args.arm_kd, args.arm_gravity_comp))
    print('Rigid TCP:', np.round(tcp.position(data), 3), 'goal:', np.round(goal.target(data), 3))
    print('Orange = target, cyan = rigid TCP.  Arm move events print [KEY] line.')

    try:
        # No key_callback here! All robot commands are owned by panel/terminal.
        with mj_viewer.launch_passive(model, data) as v:
            while v.is_running() and not io.terminate.is_set() and (not args.max_steps or tick < args.max_steps):
                wall = time.perf_counter()
                for key in io.drain():
                    if key == 'W': cmd[0] = np.clip(cmd[0] + .12, -.24, .24)
                    elif key == 'S': cmd[0] = np.clip(cmd[0] - .12, -.24, .24)
                    elif key == 'A': cmd[1] = np.clip(cmd[1] + .06, -.12, .12)
                    elif key == 'D': cmd[1] = np.clip(cmd[1] - .06, -.12, .12)
                    elif key == 'Q': cmd[2] = np.clip(cmd[2] + .12, -.30, .30)
                    elif key == 'E': cmd[2] = np.clip(cmd[2] - .12, -.30, .30)
                    elif key == ' ': cmd[:] = 0
                    elif key == 'I': goal.move(0, args.arm_step)
                    elif key == 'K': goal.move(0, -args.arm_step)
                    elif key == 'J': goal.move(1, args.arm_step)
                    elif key == 'L': goal.move(1, -args.arm_step)
                    elif key == 'U': goal.move(2, args.arm_step)
                    elif key == 'O': goal.move(2, -args.arm_step)
                    elif key == 'M': print('[MODE]', goal.toggle(data), flush=True)
                    elif key == 'T': goal.capture(data)
                    elif key == 'P': paused = not paused
                    elif key == 'X':
                        cmd[:] = 0
                        goal.capture(data)
                    if key in 'IKJLUO':
                        print('[KEY]', key, 'goal=', np.round(goal.target(data), 3),
                              'actual=', np.round(tcp.position(data), 3), flush=True)
                current_tcp = tcp.position(data)
                target = goal.target(data)
                if paused:
                    viewer_markers(v, target, current_tcp)
                    v.sync()
                    time.sleep(.02)
                    continue

                base_p = np.asarray(data.qpos[root_qa:root_qa+3], dtype=float)
                base_quat = np.asarray(data.qpos[root_qa+3:root_qa+7], dtype=float)
                y = yaw_from_quat(base_quat)
                arm_base_world = base_p + yaw_rot(y) @ arm_nominal_base_offset
                ee_obs = (body_rot(data, base_body).T @ (target - arm_base_world)).astype(np.float32)
                sample = RobotSample(
                    joint_q=np.asarray(data.qpos[qa], dtype=np.float32).copy(),
                    joint_dq=np.asarray(data.qvel[va], dtype=np.float32).copy(),
                    imu_quat_wxyz=np.asarray(data.sensor('imu_quat').data, dtype=np.float32).copy(),
                    imu_gyro_xyz=np.asarray(data.sensor('imu_gyro').data, dtype=np.float32).copy(),
                    ee_local_xyz=ee_obs,
                )
                leg_tgt, raw_action, obs = wbc.act(sample, cmd)
                arm_tgt, ik_error, arm_step_norm, singular = ik_target(
                    model, data, tcp, arm_qa, arm_va, arm_ji, target,
                    args.ik_damping, args.ik_gain, args.ik_max_joint_step, args.ik_max_error,
                )
                q_ref = np.r_[leg_tgt[:12], arm_tgt]
                tau_arm = np.zeros(6)
                for _ in range(nsub):
                    # Position PD for all joints; Z1 gains + optional bias compensation.
                    tau = kp * (q_ref - data.qpos[qa]) - kd * data.qvel[va]
                    if args.arm_gravity_comp == 'on':
                        # Computed generalized bias includes gravity/Coriolis terms.
                        # Use arm DOFs only; do not feed it into leg policy.
                        tau[12:] += data.qfrc_bias[arm_va]
                    if not np.isfinite(tau).all():
                        raise RuntimeError('Nonfinite PD torque')
                    data.ctrl[aa] = np.clip(tau, lo, hi)
                    tau_arm = np.asarray(data.ctrl[aa[12:]], dtype=float).copy()
                    mujoco.mj_step(model, data)
                actual = tcp.position(data)
                actual_error = float(np.linalg.norm(target - actual))
                if tick % 25 == 0:
                    print(f'[ARM] t={data.time:6.2f}s frame={goal.mode:5s} '
                          f'target={np.round(target,3)} actual={np.round(actual,3)} '
                          f'err={actual_error:.4f}m dqIK={arm_step_norm:.4f}rad '
                          f'sigmaMin={singular[-1]:.4f} torqueMax={np.max(np.abs(tau_arm)):.2f}Nm '
                          f'base_cmd={np.round(cmd,2)}', flush=True)
                io.set_status(f't={data.time:.2f}s  frame={goal.mode}   '
                              f'v_cmd={np.round(cmd,2)}\n'
                              f'TCP target: {np.round(target,3)}\n'
                              f'TCP actual: {np.round(actual,3)}\n'
                              f'position error: {actual_error:.4f} m\n'
                              f'IK joint step: {arm_step_norm:.4f} rad\n'
                              f'arm max torque: {np.max(np.abs(tau_arm)):.2f} Nm')
                if args.csv:
                    row = dict(t=float(data.time), vx=float(cmd[0]), vy=float(cmd[1]), wz=float(cmd[2]),
                               error=actual_error, target_x=target[0], target_y=target[1], target_z=target[2],
                               tcp_x=actual[0], tcp_y=actual[1], tcp_z=actual[2], ik_step=arm_step_norm,
                               sigma_min=singular[-1], arm_tau_max=float(np.max(np.abs(tau_arm))),
                               root_z=float(data.qpos[root_qa+2]))
                    rows.append(row)
                viewer_markers(v, goal.target(data), actual)
                v.sync()
                tick += 1
                time.sleep(max(0., cfg.control_dt - (time.perf_counter() - wall)))
    finally:
        io.terminate.set()
        if args.csv and rows:
            out = Path(args.csv).expanduser().resolve()
            out.parent.mkdir(parents=True, exist_ok=True)
            with out.open('w', newline='', encoding='utf-8') as f:
                w = csv.DictWriter(f, fieldnames=rows[0].keys())
                w.writeheader()
                w.writerows(rows)
            print('Saved:', out, flush=True)


if __name__ == '__main__':
    main()
