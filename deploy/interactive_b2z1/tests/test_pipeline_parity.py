#!/usr/bin/env python3
"""M1.4a: compare continuous policy-feedback obs/history/actions against PyTorch 1.10.

Only reads files and runs a local standalone C++ process; no DDS/SDK/hardware I/O.
"""
import argparse
from pathlib import Path
import subprocess
import sys

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--binary', type=Path, required=True)
    ap.add_argument('--onnx', type=Path, required=True)
    ap.add_argument('--frames', type=int, default=25)
    ap.add_argument('--repo-root', type=Path, default=Path(__file__).resolve().parents[3])
    args = ap.parse_args()
    if not 2 <= args.frames <= 1000:
        ap.error('--frames must be in [2,1000]')

    repo = args.repo_root.resolve()
    sys.path.insert(0, str(repo / 'b2z1_interactive_sim2sim'))
    sys.path.insert(0, str(repo / 'deploy/interactive_b2z1/tests'))
    import torch
    from lowlevel_core import LowLevelWBC, RobotSample, WBCConfig
    from test_obs_parity import sample

    cfg = WBCConfig.load(repo / 'b2z1_interactive_sim2sim/config_lowlevel.yaml')
    policy = torch.jit.load(str(repo / 'b2z1_interactive_sim2sim/b2z1_low_policy_jit.pt'),
                            map_location='cpu').eval()
    wbc = LowLevelWBC(cfg, policy)
    commands = np.array([[0., 0., 0.], [0.2, 0., 0.],
                         [0.25, -0.05, 0.1], [0., 0., 0.],
                         [-0.2, 0., 0.]], dtype=np.float32)
    py_obs, py_act = [], []
    for i in range(args.frames):
        _, action, obs = wbc.act(sample(i % 5, cfg, RobotSample), commands[i % 5])
        py_obs.append(obs)
        py_act.append(action)
    py_obs = np.stack(py_obs).astype(np.float64)
    py_act = np.stack(py_act).astype(np.float64)
    cmd = [str(args.binary.resolve()), str(args.onnx.resolve()), str(args.frames)]
    result = subprocess.run(cmd, capture_output=True, text=True, check=False)
    if result.returncode:
        raise RuntimeError(f'C++ pipeline failed with {result.returncode}: {result.stderr[:2000]}')
    if result.stderr.strip():
        print('[C++ warnings]', result.stderr[:1000])
    rows = np.array([np.fromstring(line, sep=',', dtype=np.float64)
                     for line in result.stdout.splitlines() if line.strip()])
    expected_cols = 1 + 799 + 18
    if rows.shape != (args.frames, expected_cols):
        raise AssertionError(f'Expected {(args.frames, expected_cols)} C++ rows; got {rows.shape}')
    np.testing.assert_array_equal(rows[:, 0], np.arange(args.frames))
    cpp_obs = rows[:, 1:800]
    cpp_act = rows[:, 800:818]
    delta_obs = np.abs(cpp_obs - py_obs)
    delta_action = np.abs(cpp_act - py_act)
    action_scale = np.tile([0.4, 0.45, 0.45], 4)
    target_error = delta_action[:, :12] * action_scale
    print(f'M1.4a feedback closed loop: {args.frames} frames')
    for lo, hi, label in [(0, 71, 'current71'), (71, 89, 'priv18'), (89, 799, 'hist710')]:
        print(f'  {label} max_abs={delta_obs[:, lo:hi].max():.9g}')
    print(f'  all18_action max_abs={delta_action.max():.9g}')
    print(f'  leg12_action max_abs={delta_action[:, :12].max():.9g}')
    print(f'  leg_target max_abs_rad={target_error.max():.9g}')
    # These are OFFLINE regression thresholds, NOT safe joint-motion limits.
    # Report full errors so failures can be diagnosed before tightening tolerances.
    if not np.isfinite(cpp_obs).all() or not np.isfinite(cpp_act).all():
        raise AssertionError('Non-finite C++ pipeline output')
    if float(delta_obs.max()) > 5e-4 or float(target_error.max()) > 5e-4:
        worst_i, worst_j = np.unravel_index(int(np.argmax(delta_obs)), delta_obs.shape)
        raise AssertionError(f'Closed-loop parity out of tolerance; worst obs frame={worst_i}, column={worst_j}')
    print('M1.4a CLOSED-LOOP PARITY: PASS')


if __name__ == '__main__':
    main()
