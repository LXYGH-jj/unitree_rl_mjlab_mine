#!/usr/bin/env python3
"""Compare M1 C++ builder with uploaded Python lowlevel_core.py, including history.

Usage from repo root:
python deploy/interactive_b2z1/tests/test_obs_parity.py \\
  --fixture deploy/interactive_b2z1/build/interactive_obs_fixture
"""
import argparse
from pathlib import Path
import subprocess
import sys

import numpy as np


def sample(frame, cfg, RobotSample):
    q = np.asarray(cfg.default_joint_pos, dtype=np.float32).copy()
    dq = np.empty(18, dtype=np.float32)
    for i in range(18):
        q[i] += np.float32(0.003) * np.float32((i + 1) * (frame + 1))
        dq[i] = -np.float32(0.02) * np.float32((i + 1) * (frame + 1))
    return RobotSample(
        joint_q=q,
        joint_dq=dq,
        imu_quat_wxyz=np.array([1.0 + 0.001*frame, 0.018*(frame+1),
                                -0.012*(frame+1), 0.015], dtype=np.float32),
        imu_gyro_xyz=np.array([0.1*(frame+1), -0.2*(frame+1),
                               0.03*(frame+1)], dtype=np.float32),
        ee_local_xyz=np.array([0.34-0.01*frame, -0.06+0.005*frame,
                               0.12+0.015*frame], dtype=np.float32),
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--fixture', type=Path, required=True)
    p.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[3])
    args = p.parse_args()
    repo = args.repo_root.resolve()
    ref = repo / 'b2z1_interactive_sim2sim'
    sys.path.insert(0, str(ref))
    from lowlevel_core import LowLevelWBC, RobotSample, WBCConfig
    cfg = WBCConfig.load(ref / 'config_lowlevel.yaml')
    test = LowLevelWBC(cfg, jit_policy=None)

    cmd = np.array([
        [0.,0.,0.], [0.2,0.,0.], [0.25,-0.05,0.1],
        [0.,0.,0.], [-0.2,0.,0.]
    ], dtype=np.float32)
    expected = []
    for frame in range(5):
        expected.append(test.make_obs(sample(frame, cfg, RobotSample), cmd[frame]))
        raw = np.array([np.float32(0.1)*np.float32((i+1)*(frame+1))
                        for i in range(18)], dtype=np.float32)
        if frame == 2:
            raw[0] = 125.
        test.last_action = np.clip(raw, -cfg.max_abs_action,
                                   cfg.max_abs_action)

    config_text = subprocess.check_output([str(args.fixture.resolve()), '--config'], text=True)
    actual_cfg = np.fromstring(config_text.strip(), sep=',')
    expected_cfg = np.concatenate((cfg.default_joint_pos, cfg.cmd_scale,
        np.array([cfg.control_dt, cfg.frequency_hz, cfg.dof_pos_scale,
                  cfg.dof_vel_scale, cfg.ang_vel_scale, cfg.max_abs_action])))
    np.testing.assert_allclose(actual_cfg, expected_cfg, atol=1e-6, rtol=0,
                               err_msg='C++ baseline config differs from Python YAML')

    stdout = subprocess.check_output([str(args.fixture.resolve())], text=True)
    actual = np.array([np.fromstring(line, sep=',') for line in stdout.splitlines()])
    assert actual.shape == (5, 800), actual.shape
    np.testing.assert_array_equal(actual[:,0], np.arange(5))
    reference = np.stack(expected).astype(np.float64)
    max_err = np.max(np.abs(actual[:,1:] - reference))
    print('C++ vs Python shape:', actual[:,1:].shape, reference.shape)
    print('Overall max_abs_diff:', f'{max_err:.9g}')
    for start, end, name in [(0,71,'current71'),(71,89,'priv18'),(89,799,'hist710')]:
        diff = np.abs(actual[:,1+start:1+end] - reference[:,start:end]).max()
        print(f'  {name}: {diff:.9g}')
    np.testing.assert_allclose(actual[:,1:], reference, atol=1e-5, rtol=0)
    print('M1 OBS PARITY: PASS')


if __name__ == '__main__':
    main()
