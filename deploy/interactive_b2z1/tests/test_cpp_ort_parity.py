#!/usr/bin/env python3
"""Compare existing C++ OrtRunner (ONNX Runtime 1.22) with original torch 1.10.

Read-only test; does not connect SDK2/DDS/Z1.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile

import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--binary', type=Path, required=True)
    p.add_argument('--onnx', type=Path, required=True)
    p.add_argument('--fixture', type=Path, required=True)
    p.add_argument('--reference', type=Path, required=True)
    args = p.parse_args()

    fixture_stdout = subprocess.check_output([str(args.fixture.resolve())], text=True)
    fixture_rows = [np.fromstring(s, dtype=np.float32, sep=',')
                    for s in fixture_stdout.splitlines() if s.strip()]
    assert len(fixture_rows) == 5 and all(row.shape == (800,) for row in fixture_rows), (
        "Expected five 800-column C++ fixture records"
    )

    # Temporary input CSV: keep test fixtures out of tracked source files.
    with tempfile.TemporaryDirectory(prefix='interactive_ort_m13_') as temp:
        path = Path(temp) / 'input.csv'
        path.write_text(fixture_stdout, encoding='utf-8')
        cmd = [str(args.binary.resolve()), str(args.onnx.resolve()), str(path)]
        result = subprocess.run(cmd, text=True, capture_output=True, check=False)
        if result.returncode:
            print(result.stderr)
            raise RuntimeError(f'C++ OrtRunner failed (exit {result.returncode})')
        if result.stderr.strip():
            print('[C++ runtime warnings]', result.stderr.strip()[:1000])
        rows = [np.fromstring(s, dtype=np.float32, sep=',')
                for s in result.stdout.splitlines() if s.strip()]

    got = np.stack(rows)
    reference = np.load(args.reference.resolve())
    if got.shape != reference.shape or got.shape != (5, 18):
        raise AssertionError(f'Wrong output shape: C++ {got.shape} vs torch {reference.shape}')
    if not np.isfinite(got).all():
        raise AssertionError('C++ inference produced NaN/Inf')

    delta = np.abs(got - reference)
    leg_delta = delta[:, :12]
    # Matches lowlevel_core.py policy-order leg scaling (FR, FL, RR, RL).
    scale = np.tile(np.array([0.4, 0.45, 0.45], dtype=np.float32), 4)
    leg_target_error = np.abs(np.clip(got[:, :12], -100, 100) -
                              np.clip(reference[:, :12], -100, 100)) * scale
    print('C++ ORT 1.22 vs original TorchScript 1.10')
    print('Output shapes:', got.shape, reference.shape)
    print('All 18 actions max_abs:', float(delta.max()))
    print('Leg 12 actions max_abs:', float(leg_delta.max()))
    print('Leg target max_abs (rad):', float(leg_target_error.max()))
    np.testing.assert_allclose(got, reference, atol=1e-4, rtol=1e-4)
    # Practical additional strict check on the outputs actually controlling B2.
    if float(leg_target_error.max()) > 1e-4:
        raise AssertionError('Leg target parity error exceeds 1e-4 rad')
    print('M1.3 C++ OrtRunner PARITY: PASS')


if __name__ == '__main__':
    main()
