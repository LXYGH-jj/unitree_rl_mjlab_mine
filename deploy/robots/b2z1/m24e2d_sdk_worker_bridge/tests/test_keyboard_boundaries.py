#!/usr/bin/env python3
"""Offline-only keyboard preflight regression. Does not open sockets or motors."""
import importlib.util
import math
from pathlib import Path


def load_keyboard():
    script = Path(__file__).resolve().parents[1] / "z1_bench_keyboard.py"
    spec = importlib.util.spec_from_file_location("z1_bench_keyboard", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def must_reject(fun, description):
    try:
        fun()
    except RuntimeError:
        return
    raise AssertionError(f"Expected rejection: {description}")


def run():
    kb = load_keyboard()
    # Exact REF values from the real 2026-10-16 Z1 terminal output.
    startup = [+0.000071, -0.00001180, +0.000365795,
               -0.074351, -0.000041, -0.001416]
    kb.require_valid_target_pose(startup, "ARM", startup)
    kb.require_valid_target_pose(startup, "RETURN", startup)

    j5 = startup.copy()
    j5[4] += kb.STEP_RAD
    kb.require_valid_target_pose(j5, "J5", startup)
    assert math.isclose(j5[1], startup[1]) and math.isclose(j5[2], startup[2])

    # Both slightly-outside-zero anchors may be held but NOT moved farther out.
    for idx, direction in ((1, -1), (2, +1)):
        outside = startup.copy()
        outside[idx] += direction * 0.00005
        assert not kb.allowed_bench_target(idx, outside[idx], startup)
        must_reject(lambda: kb.require_valid_target_pose(outside, "JOG", startup),
                    f"outward J{idx+1}")
        assert kb.allowed_bench_target(idx, 0.0, startup)

    # Exceptions are NOT a general widening of software travel limits.
    for idx, bad in ((1, -0.002), (2, +0.002),
                     (0, kb.WORKER_JOINT_HI[0] + 0.0001),
                     (3, kb.WORKER_JOINT_LO[3] - 0.0001),
                     (4, kb.WORKER_JOINT_HI[4] + 0.0001),
                     (5, kb.WORKER_JOINT_LO[5] - 0.0001)):
        pose = startup.copy()
        pose[idx] = bad
        must_reject(lambda: kb.require_valid_target_pose(pose, "ARM", pose),
                    f"disallowed startup J{idx+1}")

    # Exact limits remain legal.
    for idx in range(6):
        pose = list(startup)
        pose[idx] = kb.WORKER_JOINT_LO[idx]
        kb.require_valid_target_pose(pose, "INWARD", startup)
        pose[idx] = kb.WORKER_JOINT_HI[idx]
        kb.require_valid_target_pose(pose, "INWARD", startup)

    print("KEYBOARD OFFLINE PASS: real J2/J3 startup ARM and J5 jog valid; "
          "outward movement and larger limit violations rejected.")
    print("NO SDK, motors, or Worker started.")


if __name__ == "__main__":
    run()
