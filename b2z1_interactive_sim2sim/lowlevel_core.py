"""ALORE/Interactive B2+Z1 LOW-LEVEL WBC, independent of upper-level policy.

No ROS, CycloneDDS, Unitree SDK or hardware actuation in this module.
Observation layout derives from ALORE Deployment/B2_deploy/deploy_real_b2z1_obj.py
and Interactive low_level_WBC/.../manip_loco.py.
"""
from dataclasses import dataclass
from pathlib import Path
from typing import Optional
import numpy as np

PROPRIO_DIM = 71
PRIV_DIM = 18
HISTORY_LEN = 10
OBS_DIM = PROPRIO_DIM * (HISTORY_LEN + 1) + PRIV_DIM
ACTION_DIM = 18


@dataclass
class WBCConfig:
    # Position in the *policy/deployment order*: FR, FL, RR, RL, joint1..joint6.
    default_joint_pos: np.ndarray
    action_scale: np.ndarray
    leg_min: np.ndarray
    leg_max: np.ndarray
    cmd_scale: np.ndarray
    control_dt: float = 0.02
    dof_pos_scale: float = 1.0
    dof_vel_scale: float = 0.05
    ang_vel_scale: float = 0.25
    frequency_hz: float = 2.0
    startup_hold_cycles: int = 10
    max_abs_action: float = 100.0  # Original Interactive clip_actions=100.0; NOT a safety limit.

    @classmethod
    def load(cls, path):
        import yaml
        cfg = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
        low = cfg['low_level']
        scale = np.asarray(low['action_scale'], dtype=np.float32)
        default = np.asarray(low['default_angles'], dtype=np.float32)
        if scale.shape != (ACTION_DIM,) or default.shape != (ACTION_DIM,):
            raise ValueError('low_level action_scale and default_angles must each have 18 elements')
        return cls(
            default_joint_pos=default,
            action_scale=scale,
            leg_min=np.asarray(cfg['joint_limits_min'], dtype=np.float32),
            leg_max=np.asarray(cfg['joint_limits_max'], dtype=np.float32),
            cmd_scale=np.asarray(low['cmd_scale'], dtype=np.float32),
            control_dt=float(cfg['control_dt']),
            dof_pos_scale=float(low['dof_pos_scale']),
            dof_vel_scale=float(low['dof_vel_scale']),
            ang_vel_scale=float(low['ang_vel_scale']),
            frequency_hz=float(low.get('frequency_hz', 2.0)),
            startup_hold_cycles=int(low.get('startup_hold_cycles', 10)),
            max_abs_action=float(low.get('max_abs_action', 100.0)),
        )

    def __post_init__(self):
        for name, arr, n in (
            ('default_joint_pos', self.default_joint_pos, 18),
            ('action_scale', self.action_scale, 18),
            ('leg_min', self.leg_min, 12), ('leg_max', self.leg_max, 12),
            ('cmd_scale', self.cmd_scale, 3),
        ):
            a = np.asarray(arr)
            if a.shape != (n,) or not np.isfinite(a).all():
                raise ValueError('%s must be finite array[%d]' % (name, n))
        if np.any(self.leg_min >= self.leg_max):
            raise ValueError('invalid leg joint limits')
        if self.control_dt <= 0 or self.frequency_hz <= 0:
            raise ValueError('invalid control_dt / frequency_hz')


@dataclass
class RobotSample:
    # joint_q/dq order is FR FL RR RL + Z1 joint1..6 (18 total); arm gripper excluded.
    joint_q: np.ndarray
    joint_dq: np.ndarray
    imu_quat_wxyz: np.ndarray
    imu_gyro_xyz: np.ndarray
    # This is the observation item, NOT automatically the measured EE position!
    # For Interactive parity it should be the desired EE goal expressed as in
    # Interactive: inverse(base_quat) * (EE_goal_world - arm_base_world).
    ee_local_xyz: np.ndarray


def body_roll_pitch_wxyz(q):
    q = np.asarray(q, dtype=np.float64).reshape(4)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or not 0.9 <= n <= 1.1:
        raise ValueError('invalid IMU quaternion norm; reject state rather than normalizing invalid IMU data')
    w, x, y, z = q / n
    roll = np.arctan2(2. * (w * x + y * z), 1. - 2. * (x * x + y * y))
    pitch = np.arcsin(np.clip(2. * (w * y - z * x), -1., 1.))
    return np.asarray([roll, pitch], dtype=np.float32)


class LowLevelWBC:
    """Pure observation+policy+target computation. Does NOT control motors.

    ALORE parity: obs history is concatenated BEFORE advancing the 10-frame
    FIFO; on first step it therefore contains zero history.
    """
    def __init__(self, cfg: WBCConfig, jit_policy):
        self.cfg = cfg
        self.policy = jit_policy
        self.reset()

    def reset(self):
        self.step_count = 0
        self.gait_phase = 0.
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.hist = np.zeros((HISTORY_LEN, PROPRIO_DIM), dtype=np.float32)
        self.last_obs = None
        self.last_proprio = None

    def _gait(self, cmd):
        # ALORE: walking if any command component > 0.1; rate=50Hz, freq=2Hz
        self.gait_phase = (self.gait_phase + self.cfg.control_dt * self.cfg.frequency_hz) % 1.0
        if not np.any(np.abs(cmd) > 0.1):
            self.gait_phase = 0.0
        clock = np.sin(2 * np.pi * (self.gait_phase + np.array([0.5, 0., 0., 0.5])))
        return np.float32(self.gait_phase), clock.astype(np.float32)

    def make_obs(self, s: RobotSample, cmd):
        c = np.asarray(cmd, dtype=np.float32).reshape(-1)
        if c.shape != (3,) or not np.isfinite(c).all():
            raise ValueError('cmd must be finite [vx,vy,wz]')
        def arr(label, v, n):
            v = np.asarray(v, dtype=np.float32)
            if v.shape != (n,) or not np.isfinite(v).all():
                raise ValueError(f'{label} must be finite [{n}]')
            return v
        q = arr('joint_q', s.joint_q, 18)
        dq = arr('joint_dq', s.joint_dq, 18)
        gyro = arr('imu_gyro_xyz', s.imu_gyro_xyz, 3)
        ee = arr('ee_local_xyz', s.ee_local_xyz, 3)
        phase, clock = self._gait(c)
        p = np.zeros(PROPRIO_DIM, dtype=np.float32)
        p[0:2] = body_roll_pitch_wxyz(s.imu_quat_wxyz)
        p[2:5] = gyro * self.cfg.ang_vel_scale
        p[5:23] = (q - self.cfg.default_joint_pos) * self.cfg.dof_pos_scale
        p[23:41] = dq * self.cfg.dof_vel_scale
        p[41:53] = self.last_action[:12]
        # p[53:57] foot contacts zeroed in Interactive and ALORE
        p[57:60] = c * self.cfg.cmd_scale
        p[60:63] = ee
        # p[63:66] orientation placeholder = 0
        p[66] = phase
        p[67:71] = clock
        # ALORE initializes friction-related privileged placeholder to 1.0.
        priv = np.zeros(PRIV_DIM, dtype=np.float32)
        priv[5] = 1.0
        obs = np.concatenate((p, priv, self.hist.flatten()), axis=0).astype(np.float32)
        assert obs.shape == (OBS_DIM,)
        if self.step_count == 0:
            self.hist[:] = p
        else:
            self.hist[:-1] = self.hist[1:]
            self.hist[-1] = p
        self.last_obs, self.last_proprio = obs, p.copy()
        self.step_count += 1
        return obs

    def act(self, s: RobotSample, cmd):
        """Returns (target_q18, raw_action18, observation799), no I/O side effects."""
        import torch
        obs = self.make_obs(s, cmd)
        with torch.no_grad():
            out = self.policy(torch.from_numpy(obs).reshape(1, -1))
        if isinstance(out, (tuple, list)):
            raise ValueError('Expected one Tensor output from history-encoding TorchScript actor')
        raw = out.detach().cpu().numpy().reshape(-1).astype(np.float32)
        if raw.shape != (ACTION_DIM,) or not np.isfinite(raw).all():
            raise ValueError(f'policy must return finite action[18], got {raw.shape}')
        self.last_action = np.clip(raw, -self.cfg.max_abs_action, self.cfg.max_abs_action)
        if self.step_count <= self.cfg.startup_hold_cycles:
            target = self.cfg.default_joint_pos.copy()
        else:
            target = self.cfg.default_joint_pos + self.last_action * self.cfg.action_scale
        target[:12] = np.clip(target[:12], self.cfg.leg_min, self.cfg.leg_max)
        if not np.isfinite(target).all():
            raise ValueError('non-finite target')
        return target, self.last_action.copy(), obs
