#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""B2+Z1 Interactive locomotion + position-only IK: MuJoCo demo.

SIMULATION ONLY. Uses the existing lowlevel_core.py and Interactive TorchScript actor.
The Z1 policy action slots 12:18 are intentionally not actuated; arm follows IK.
The commanded EE target is used in observation[60:63] via the Interactive mapping.
"""
from __future__ import annotations

import argparse
import csv
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import mujoco
import numpy as np
import torch
from mujoco import viewer as mj_viewer

LEG_PREFIXES = ("FR", "FL", "RR", "RL")  # Interactive policy order
LEG_NAMES = tuple(f"{leg}_{name}_joint" for leg in LEG_PREFIXES for name in ("hip", "thigh", "calf"))
ARM_NAMES = tuple(f"joint{i}" for i in range(1, 7))
JOINT_NAMES = LEG_NAMES + ARM_NAMES


def get_joint_actuator_indices(m):
    qadr, vadr, aid, jid = [], [], [], []
    for name in JOINT_NAMES:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        an = name[:-6] if name.endswith("_joint") else name
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, an)
        if j < 0 or a < 0:
            raise RuntimeError(f"Missing MuJoCo joint/actuator: {name} / {an}")
        jid.append(j)
        qadr.append(m.jnt_qposadr[j]); vadr.append(m.jnt_dofadr[j]); aid.append(a)
    return (np.asarray(qadr, dtype=int), np.asarray(vadr, dtype=int),
            np.asarray(aid, dtype=int), np.asarray(jid, dtype=int))


def rotmat(m, body_id):
    return np.asarray(m.xmat[body_id]).reshape(3, 3)


def yaw_from_quat(q):
    w, x, y, z = [float(n) for n in q]
    return math.atan2(2 * (w*z+x*y), 1 - 2*(y*y+z*z))


def yaw_matrix(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


class Keyboard:
    """Key callback runs in viewer thread, commands are consumed in physics thread."""
    def __init__(self):
        self.queue = deque()
        self.mutex = threading.Lock()

    def callback(self, code):
        try:
            key = chr(code).upper()
        except (ValueError, OverflowError):
            return
        if key in "WSADQE IJKLUOMTRCP":
            with self.mutex:
                self.queue.append(key)

    def pop_all(self):
        with self.mutex:
            items = list(self.queue)
            self.queue.clear()
        return items


class Goal:
    def __init__(self, m, d, mount_id, tcp_id):
        self.mount_id = mount_id
        self.tcp_id = tcp_id
        self.mode = "body"                 # body-mounted goal by default
        self.local = np.zeros(3)
        self.world = np.zeros(3)
        self.capture_tcp(d)

    def mount_pos_rot(self, d):
        return np.asarray(d.xpos[self.mount_id], dtype=float), rotmat(d, self.mount_id)

    def capture_tcp(self, d):
        p, R = self.mount_pos_rot(d)
        self.world = np.asarray(d.xpos[self.tcp_id], dtype=float).copy()
        self.local = R.T @ (self.world - p)

    def target_world(self, d):
        p, R = self.mount_pos_rot(d)
        if self.mode == "body":
            return p + R @ self.local
        return self.world.copy()

    def toggle(self, d):
        w = self.target_world(d)
        p, R = self.mount_pos_rot(d)
        self.mode = "world" if self.mode == "body" else "body"
        self.local = R.T @ (w - p)
        self.world = w
        return self.mode

    def increment(self, d, axis, delta):
        if self.mode == "body":
            self.local[axis] += delta
        else:
            self.world[axis] += delta


def ik_step(m, d, tcp_id, arm_vadr, arm_qadr, arm_jid, goal_world,
            damping, gain, max_step, max_error):
    """Damped least-squares position IK; updates a POSITION target, not qpos.

    Jacobian includes base DOFs, but ONLY the six Z1 columns are used.
    """
    current_tcp = np.asarray(d.xpos[tcp_id], dtype=float).copy()
    err = np.asarray(goal_world, dtype=float) - current_tcp
    err_norm = float(np.linalg.norm(err))
    if err_norm > max_error:
        # Avoid an infeasible world-fixed target causing arbitrarily large demand.
        err *= max_error / max(err_norm, 1e-12)
    jacp = np.zeros((3, m.nv), dtype=float)
    jacr = np.zeros((3, m.nv), dtype=float)
    mujoco.mj_jacBody(m, d, jacp, jacr, tcp_id)
    J = jacp[:, arm_vadr]
    # dq = J^T (J J^T + lambda^2 I)^(-1) e
    H = J @ J.T + (damping*damping)*np.eye(3)
    dq = gain * J.T @ np.linalg.solve(H, err)
    dq = np.clip(dq, -max_step, max_step)
    q_target = np.asarray(d.qpos[arm_qadr], dtype=float) + dq
    lower = np.asarray(m.jnt_range[arm_jid, 0], dtype=float)
    upper = np.asarray(m.jnt_range[arm_jid, 1], dtype=float)
    limits = np.asarray(m.jnt_limited[arm_jid], dtype=bool)
    q_target = np.where(limits, np.clip(q_target, lower, upper), q_target)
    if not np.isfinite(q_target).all():
        raise RuntimeError("IK generated nonfinite arm target")
    return q_target, err_norm


def draw_goal(viewer, goal_world):
    """Draw a small orange sphere at requested TCP position."""
    try:
        if viewer.user_scn.maxgeom <= 0:
            return
        viewer.user_scn.ngeom = 1
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[0],
            mujoco.mjtGeom.mjGEOM_SPHERE,
            [0.025, 0, 0],
            np.asarray(goal_world, dtype=float),
            np.eye(3).reshape(-1),
            [1, 0.45, 0.05, 0.9],
        )
    except (AttributeError, TypeError, ValueError):
        # Visualization is not required for controlling the robot.
        pass


def main():
    ap = argparse.ArgumentParser(description="B2+Z1 MuJoCo Interactive RL + keyboard TCP IK")
    ap.add_argument("--xml", required=True, help="B2Z1 scene XML from unitree_rl_mjlab_mine")
    ap.add_argument("--policy", required=True, help="Interactive b2z1_low_policy_jit.pt")
    ap.add_argument("--wbc-dir", required=True, help="Folder containing lowlevel_core.py and config_lowlevel.yaml")
    ap.add_argument("--config", default=None)
    ap.add_argument("--physics-dt", type=float, default=0.005)
    ap.add_argument("--initial-z", type=float, default=0.6)
    ap.add_argument("--arm-kp", type=float, default=60.0)
    ap.add_argument("--arm-kd", type=float, default=4.0)
    ap.add_argument("--arm-step", type=float, default=0.015, help="EE target increment per key event, m")
    ap.add_argument("--ik-damping", type=float, default=0.07)
    ap.add_argument("--ik-gain", type=float, default=0.65)
    ap.add_argument("--ik-max-joint-step", type=float, default=0.035, help="rad/control tick")
    ap.add_argument("--ik-max-error", type=float, default=0.20, help="m; clamp IK input error")
    ap.add_argument("--max-steps", type=int, default=0, help="0 means until viewer closes")
    ap.add_argument("--csv", default="", help="Optional live control log CSV")
    args = ap.parse_args()
    if args.arm_step <= 0 or args.ik_damping <= 0 or args.ik_max_joint_step <= 0:
        ap.error("IK tuning parameters must be positive")

    sys.path.insert(0, str(Path(args.wbc_dir).expanduser().resolve()))
    from lowlevel_core import LowLevelWBC, RobotSample, WBCConfig

    cfg_path = Path(args.config or Path(args.wbc_dir) / "config_lowlevel.yaml")
    cfg = WBCConfig.load(cfg_path)
    cfg.startup_hold_cycles = 0
    # Keep original Gym behavior for policy joint TARGET (no extra target angle clamp).
    # Simulation-only. Real robot must have safety-constrained targets.
    cfg.leg_min[:] = -1e6; cfg.leg_max[:] = +1e6
    policy = torch.jit.load(str(Path(args.policy).resolve()), map_location="cpu").eval()
    wbc = LowLevelWBC(cfg, policy)

    model = mujoco.MjModel.from_xml_path(str(Path(args.xml).resolve()))
    model.opt.timestep = args.physics_dt
    nsub = int(round(cfg.control_dt / model.opt.timestep))
    if nsub < 1 or abs(nsub*model.opt.timestep-cfg.control_dt) > 1e-8:
        ap.error("control_dt should be a multiple of physics-dt")
    data = mujoco.MjData(model)
    home = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home >= 0: mujoco.mj_resetDataKeyframe(model, data, home)
    else: mujoco.mj_resetData(model, data)

    qa, va, aa, ji = get_joint_actuator_indices(model)
    root_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
    mount_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link00")
    tcp_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "tcp")
    if min(root_joint, mount_id, tcp_id) < 0:
        raise RuntimeError("Model needs floating_base_joint, link00 and tcp")
    root_qa = int(model.jnt_qposadr[root_joint])
    data.qpos[root_qa+2] = args.initial_z
    data.qpos[qa] = cfg.default_joint_pos
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)

    # Do NOT change the effective pose of the arm when the demo starts.
    goal = Goal(model, data, mount_id, tcp_id)
    cmd = np.zeros(3, dtype=np.float32)
    kp = np.r_[np.full(12, 360.), np.full(6, args.arm_kp)]
    kd = np.r_[np.full(12, 5.), np.full(6, args.arm_kd)]
    act_min = model.actuator_ctrlrange[aa, 0].copy()
    act_max = model.actuator_ctrlrange[aa, 1].copy()
    arm_qa, arm_va, arm_ji = qa[12:], va[12:], ji[12:]
    arm_base_offset = np.array([.20, 0, .09], dtype=float) # Interactive nominal definition
    kb = Keyboard()
    paused = False
    tick_no = 0
    csv_rows = []

    print("\n== B2+Z1 MuJoCo keyboard teleoperation (simulation only) ==")
    print("Leg: W/S vx, A/D vy, Q/E wz; SPACE stop")
    print("TCP: I/K +X/-X, J/L +Y/-Y, U/O +Z/-Z")
    print("M switch TCP goal frame (body/world); T retarget current TCP; P pause")
    print("Click the MuJoCo viewer window first; close it to exit.")
    print("Initial TCP (world):", np.round(data.xpos[tcp_id], 3))
    print("Arm offset for policy observation:", arm_base_offset)

    with mj_viewer.launch_passive(model, data, key_callback=kb.callback) as v:
        while v.is_running() and (args.max_steps <= 0 or tick_no < args.max_steps):
            wall_start = time.perf_counter()
            for key in kb.pop_all():
                if key == "W": cmd[0] = np.clip(cmd[0]+0.12, -0.24, 0.24)
                elif key == "S": cmd[0] = np.clip(cmd[0]-0.12, -0.24, 0.24)
                elif key == "A": cmd[1] = np.clip(cmd[1]+0.06, -0.12, 0.12)
                elif key == "D": cmd[1] = np.clip(cmd[1]-0.06, -0.12, 0.12)
                elif key == "Q": cmd[2] = np.clip(cmd[2]+0.12, -0.30, 0.30)
                elif key == "E": cmd[2] = np.clip(cmd[2]-0.12, -0.30, 0.30)
                elif key == " ": cmd[:] = 0.0
                elif key == "I": goal.increment(data, 0, args.arm_step)
                elif key == "K": goal.increment(data, 0, -args.arm_step)
                elif key == "J": goal.increment(data, 1, args.arm_step)
                elif key == "L": goal.increment(data, 1, -args.arm_step)
                elif key == "U": goal.increment(data, 2, args.arm_step)
                elif key == "O": goal.increment(data, 2, -args.arm_step)
                elif key == "T": goal.capture_tcp(data)
                elif key == "M": print("[TCP frame]", goal.toggle(data), flush=True)
                elif key == "P": paused = not paused
            if paused:
                draw_goal(v, goal.target_world(data)); v.sync()
                time.sleep(0.02)
                continue

            target_world = goal.target_world(data)
            base_pos = np.asarray(data.qpos[root_qa:root_qa+3], dtype=float)
            base_quat = np.asarray(data.qpos[root_qa+3:root_qa+7], dtype=float)
            yaw = yaw_from_quat(base_quat)
            base_R = rotmat(data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link"))
            arm_base_world = base_pos + yaw_matrix(yaw) @ arm_base_offset
            ee_obs = (base_R.T @ (target_world - arm_base_world)).astype(np.float32)
            sample = RobotSample(
                joint_q=np.asarray(data.qpos[qa], dtype=np.float32).copy(),
                joint_dq=np.asarray(data.qvel[va], dtype=np.float32).copy(),
                imu_quat_wxyz=np.asarray(data.sensor("imu_quat").data, dtype=np.float32).copy(),
                imu_gyro_xyz=np.asarray(data.sensor("imu_gyro").data, dtype=np.float32).copy(),
                ee_local_xyz=ee_obs,
            )
            leg_target, action, obs = wbc.act(sample, cmd)
            # The last 6 RL outputs are not motor commands. Use the same goal for arm IK
            # and the RL EE observation to avoid contradictory target definitions.
            arm_target, err_norm = ik_step(
                model, data, tcp_id, arm_va, arm_qa, arm_ji, target_world,
                args.ik_damping, args.ik_gain, args.ik_max_joint_step, args.ik_max_error,
            )
            q_ref = np.r_[leg_target[:12], arm_target]
            for _ in range(nsub):
                torque = kp * (q_ref - data.qpos[qa]) - kd * data.qvel[va]
                data.ctrl[aa] = np.clip(torque, act_min, act_max)
                mujoco.mj_step(model, data)

            if tick_no % 25 == 0:
                p = np.asarray(data.qpos[root_qa:root_qa+3], dtype=float)
                print(f"t={data.time:.2f} cmd=[{cmd[0]:+.2f},{cmd[1]:+.2f},{cmd[2]:+.2f}] "
                      f"root_z={p[2]:.3f} yaw={math.degrees(yaw):+.1f}deg "
                      f"TCP_err={err_norm:.3f}m frame={goal.mode} "
                      f"EE_obs={np.round(ee_obs,3)}", flush=True)
            if args.csv:
                tcp = np.asarray(data.xpos[tcp_id], dtype=float).copy()
                csv_rows.append(dict(t=float(data.time),vx=float(cmd[0]),vy=float(cmd[1]),wz=float(cmd[2]),
                                     root_z=float(data.qpos[root_qa+2]),
                                     tcp_err=float(np.linalg.norm(target_world-tcp)),
                                     tcp_x=float(tcp[0]),tcp_y=float(tcp[1]),tcp_z=float(tcp[2]),
                                     target_x=float(target_world[0]),target_y=float(target_world[1]),target_z=float(target_world[2])))
            draw_goal(v, goal.target_world(data))
            v.sync()
            tick_no += 1
            time.sleep(max(0, cfg.control_dt-(time.perf_counter()-wall_start)))

    if args.csv and csv_rows:
        path = Path(args.csv).expanduser().resolve()
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as f:
            wr = csv.DictWriter(f, fieldnames=list(csv_rows[0]))
            wr.writeheader(); wr.writerows(csv_rows)
        print("Saved:", path)


if __name__ == "__main__":
    main()
