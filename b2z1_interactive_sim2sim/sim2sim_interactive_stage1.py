#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Stage-1 Interactive-to-MuJoCo reference test: same actor and *static-arm* protocol.

Require a companion Gym test with static arm target. No ROS/DDS/real motors.
This does not claim full Isaac Gym physics parity. It does change the original runner:
- first-step policy action is applied immediately, no 10-tick hold;
- Z1 arm joint PD uses original Gym Kp=5, Kd=0.5;
- default initial body z=0.6 m rather than XML keyframe 0.5m;
- EE target observation follows Gym teleop spherical goal as robot moves;
- joint mapping, policy and action scaling still use existing lowlevel_core.
"""
import argparse
import csv
import math
import time
from pathlib import Path

import mujoco
import numpy as np
import torch

from lowlevel_core import WBCConfig, LowLevelWBC, RobotSample

LEG_PREFIXES = ('FR','FL','RR','RL')
LEG_JOINTS = tuple(f'{l}_{p}_joint' for l in LEG_PREFIXES for p in ('hip','thigh','calf'))
JOINTS = LEG_JOINTS + tuple(f'joint{i}' for i in range(1,7))


def ids_for_model(model):
    qa, da, aa = [], [], []
    for name in JOINTS:
        j=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,name)
        a_name = name[:-6] if name.endswith('_joint') else name
        a=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_ACTUATOR,a_name)
        if j<0 or a<0:
            raise RuntimeError(f'缺少关节或电机: {name}, actuator={a_name}')
        qa.append(int(model.jnt_qposadr[j]))
        da.append(int(model.jnt_dofadr[j]))
        aa.append(int(a))
    assert len(set(aa))==18
    return np.asarray(qa),np.asarray(da),np.asarray(aa)


def yaw_from_quat(q):
    w,x,y,z=np.asarray(q,dtype=float)
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))


def quat_inverse_apply(q,v):
    w,x,y,z=np.asarray(q,dtype=float)
    b=np.array([x,y,z],dtype=float)
    cross=-2*np.cross(b,v)
    return np.asarray(v,dtype=float)+w*cross+np.cross(-b,cross)


def rotate_yaw(yaw,v):
    c,s=math.cos(yaw),math.sin(yaw)
    x,y,z=v
    return np.array([c*x-s*y,s*x+c*y,z],dtype=float)


def gym_teleop_ee_goal(data):
    """Derived from Interactive ManipLoco._update_curr_ee_goal/compute_observations:

    teleop goal sphere starts [radius=0.5, pitch=pi/8, yaw=0]; center is
    at [root_x,root_y,0] + Ryaw*[0.2,0,0.7]. arm_base_world is
    root_pos + Ryaw*[0.2,0,0.09]. EE observation is Rbase^-1*(world_goal-arm_base_world).
    (The spherical pitch sign is set to match IsaacGym sphere2cart convention;
     compare the first EE observation against Gym before treating this as exact parity.)
    """
    base_p=np.array(data.qpos[:3],dtype=float)
    quat=np.array(data.qpos[3:7],dtype=float)
    yaw=yaw_from_quat(quat)
    pitch=math.pi/8
    sphere=np.array([0.5*math.cos(pitch),0.0,0.5*math.sin(pitch)])
    world_goal=np.array([base_p[0],base_p[1],0.0]) + rotate_yaw(yaw,np.array([0.2,0,0.7])+sphere)
    arm_base_world=base_p + rotate_yaw(yaw,np.array([0.2,0,0.09]))
    return quat_inverse_apply(quat,world_goal-arm_base_world).astype(np.float32)


def fixed_initial_tcp_goal(model,data):
    ee_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,'tcp')
    if ee_id<0: raise ValueError('缺少tcp body')
    offset=np.array([0.2,0,0.09])
    return quat_inverse_apply(data.qpos[3:7], data.xpos[ee_id]-data.qpos[:3]-offset).astype(np.float32)


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--xml',required=True)
    ap.add_argument('--policy',required=True)
    ap.add_argument('--config',default=str(Path(__file__).with_name('config_lowlevel.yaml')))
    ap.add_argument('--vx',type=float,default=.3)
    ap.add_argument('--seconds',type=float,default=6.)
    ap.add_argument('--warmup',type=float,default=1.)
    ap.add_argument('--physics-dt',type=float,default=.005)
    ap.add_argument('--initial-z',type=float,default=.6)
    ap.add_argument('--ee-mode',choices=['gym_teleop','initial_tcp'],default='gym_teleop')
    ap.add_argument('--csv',default='mujoco_stage1.csv')
    ap.add_argument('--viewer',action='store_true')
    args=ap.parse_args()
    if not (0 < args.seconds <= 30 and 0 <= args.warmup < args.seconds and 0 < args.physics_dt <= .02 and abs(args.vx) <= .6):
        ap.error('参数超出第一阶段离线测试范围')

    cfg=WBCConfig.load(args.config)
    cfg.startup_hold_cycles=0 # Original Gym policy actions have no 10-tick hold
    # Original Gym clips TORQUE, not the policy joint-angle target.
    # Disable the extra target clamp from previous deployment core for SIM-ONLY parity.
    cfg.leg_min[:] = -1e6
    cfg.leg_max[:] = +1e6
    net=torch.jit.load(str(Path(args.policy).resolve()),map_location='cpu').eval()
    wbc=LowLevelWBC(cfg,net)
    model=mujoco.MjModel.from_xml_path(str(Path(args.xml).resolve()))
    model.opt.timestep=args.physics_dt
    substeps=int(round(cfg.control_dt/model.opt.timestep))
    assert substeps>=1 and abs(substeps*model.opt.timestep-cfg.control_dt)<1e-8
    data=mujoco.MjData(model)
    home=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_KEY,'home')
    if home>=0: mujoco.mj_resetDataKeyframe(model,data,home)
    else: mujoco.mj_resetData(model,data)
    qa,da,aa=ids_for_model(model)
    base_id=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_JOINT,'floating_base_joint')
    if base_id<0: raise ValueError('缺少 floating_base_joint')
    base_qa=int(model.jnt_qposadr[base_id])
    data.qpos[base_qa+2]=args.initial_z
    data.qpos[qa]=cfg.default_joint_pos
    data.qvel[:]=0
    mujoco.mj_forward(model,data)
    ee_fixed=fixed_initial_tcp_goal(model,data) if args.ee_mode=='initial_tcp' else None
    print('MuJoCo initial EE input:', gym_teleop_ee_goal(data) if ee_fixed is None else ee_fixed)

    kp=np.array([360.]*12+[5.]*6)
    kd=np.array([5.]*12+[.5]*6)
    # Match Interactive source's leg PD and arm position-servo *gains*. Arm is held
    # at default position in BOTH engines in this static-arm A/B protocol.
    ctrl_min=model.actuator_ctrlrange[aa,0].copy()
    ctrl_max=model.actuator_ctrlrange[aa,1].copy()
    rows=[]
    prev_p=np.asarray(data.qpos[base_qa:base_qa+3],dtype=float).copy()
    viewer=None
    if args.viewer:
        from mujoco import viewer as mj_viewer
        viewer=mj_viewer.launch_passive(model,data)
    try:
        nsteps=int(round(args.seconds/cfg.control_dt))
        for k in range(nsteps):
            tick=time.perf_counter()
            t0=k*cfg.control_dt
            cmd=np.array([0.0 if t0 < args.warmup else args.vx,0.,0.],dtype=np.float32)
            ee= gym_teleop_ee_goal(data) if ee_fixed is None else ee_fixed
            sample=RobotSample(
                joint_q=data.qpos[qa].astype(np.float32).copy(),
                joint_dq=data.qvel[da].astype(np.float32).copy(),
                imu_quat_wxyz=np.array(data.sensor('imu_quat').data,dtype=np.float32),
                imu_gyro_xyz=np.array(data.sensor('imu_gyro').data,dtype=np.float32),
                ee_local_xyz=ee,
            )
            with torch.no_grad():
                tgt,actions,obs=wbc.act(sample,cmd)
            assert obs.size==799 and actions.size==18
            tgt=np.array(tgt,dtype=float)
            tgt[12:]=cfg.default_joint_pos[12:] # static-arm Gym companion matches this
            for _ in range(substeps):
                tau=kp*(tgt-data.qpos[qa])-kd*data.qvel[da]
                if not np.isfinite(tau).all(): raise RuntimeError('力矩非有限值')
                data.ctrl[aa]=np.clip(tau,ctrl_min,ctrl_max)
                mujoco.mj_step(model,data)
            t=(k+1)*cfg.control_dt
            p=np.asarray(data.qpos[base_qa:base_qa+3],dtype=float).copy()
            q=np.asarray(data.qpos[base_qa+3:base_qa+7],dtype=float)
            yaw=yaw_from_quat(q)
            vel=(p-prev_p)/cfg.control_dt
            prev_p=p
            bx=math.cos(yaw)*vel[0]+math.sin(yaw)*vel[1]
            by=-math.sin(yaw)*vel[0]+math.cos(yaw)*vel[1]
            row=dict(t=t,cmd_vx=float(cmd[0]),x=float(p[0]),y=float(p[1]),z=float(p[2]),
                     yaw_deg=math.degrees(yaw),world_vx=float(vel[0]),world_vy=float(vel[1]),
                     body_vx=float(bx),body_vy=float(by),ee_goal_x=float(ee[0]),ee_goal_y=float(ee[1]),ee_goal_z=float(ee[2]))
            rows.append(row)
            if k%25==0 or k==nsteps-1:
                print(f't={t:.2f}s cmd_vx={cmd[0]:+.2f} x={p[0]:+.3f} y={p[1]:+.3f} z={p[2]:.3f} '
                      f'yaw={row["yaw_deg"]:+.2f}deg body_vx={bx:+.3f} body_vy={by:+.3f} ee={ee}',flush=True)
            if viewer is not None:
                if not viewer.is_running():break
                viewer.sync()
            if args.viewer: time.sleep(max(0.,cfg.control_dt-(time.perf_counter()-tick)))
        if rows:
            out=Path(args.csv).resolve()
            out.parent.mkdir(parents=True,exist_ok=True)
            with out.open('w',newline='',encoding='utf-8') as f:
                w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
            steady=[r for r in rows if args.warmup+1.5<=r['t']<=args.seconds-.1]
            if not steady: steady=[r for r in rows if r['t']>=args.warmup+.5]
            avg=lambda name:sum(r[name] for r in steady)/len(steady)
            print('===== MuJoCo static-arm stage1 averages =====')
            print(f'target={args.vx:.3f} avg_body_vx={avg("body_vx"):.3f} avg_body_vy={avg("body_vy"):.3f} '
                  f'avg_root_z={avg("z"):.3f} final_yaw={rows[-1]["yaw_deg"]:+.2f}deg')
            print(f'CSV: {out}')
    finally:
        if viewer is not None:viewer.close()

if __name__=='__main__':
    main()
