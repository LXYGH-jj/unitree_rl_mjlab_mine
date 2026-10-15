#!/usr/bin/env python3
import importlib.util
from pathlib import Path

p = Path(__file__).resolve().parents[1]/'tools/analyze_sim_overlap.py'
spec=importlib.util.spec_from_file_location('analyze_sim_overlap',p)
module=importlib.util.module_from_spec(spec)
import sys
sys.modules[spec.name]=module
spec.loader.exec_module(module)

def line(seconds, content):
    return '[2026-10-14 23:00:%02d.000] [info] %s' % (seconds, content)

def assert_result():
    basic=[line(0, 'FSM: Change state from FixStand to JointCtrlMockSim'),
           line(1, '[M24d5] SIM mock gate sent=99 measured_q0=0.0000 gated_target_q0=0.0000 (NO REAL Z1 IO)'),
           line(2, '[M20a] W vx=0.12 at sim tick=10000ms'),
           line(2, '[M24d5] SIM mock gate sent=199 measured_q0=0.0000 gated_target_q0=0.0000 (NO REAL Z1 IO)'),
           line(3, '[M20diag] sim_ms=11000 vx=0.12 policy_steps=100 phase=0.1200 sat=0 leg_lag=0.1100 arm_track=0.0630 tcp_err_m=0.0520 arm_lock=false xyz=true rejected=0 misses=0'),
           line(3, '[M20a] IK TARGET XYZ_B=[0.4500,0.0100,0.3300] fk_err=0.000000'),
           line(3, '[M24d5] SIM mock gate sent=399 measured_q0=0.0410 gated_target_q0=0.0440 (NO REAL Z1 IO)'),
           line(4, '[M20a] max forward 2000ms reached; vx=0')]
    result=module.analyze('\n'.join(basic))
    assert result['overall']=='PASS_OUTPUT_EVIDENCE', result
    assert result['walk_episodes'][0]['ik_targets_during_walk']==1
    after=basic.copy()
    after.remove(line(3, '[M20a] IK TARGET XYZ_B=[0.4500,0.0100,0.3300] fk_err=0.000000'))
    after.append(line(5, '[M20a] IK TARGET XYZ_B=[0.4500,0.0100,0.3300] fk_err=0.000000'))
    no=module.analyze('\n'.join(after))
    assert no['overall']=='NO_SIMULTANEOUS_EVIDENCE',no
    bad=module.analyze('\n'.join(basic + [line(4,'[M20a] SIM FAULT latched')]))
    assert bad['overall']=='FAULT',bad
    insufficient=module.analyze('\n'.join(basic[:2]))
    assert insufficient['overall']=='NO_SIMULTANEOUS_EVIDENCE',insufficient
    print('M2.4d6 synthetic positive, after-walk negative, fault and missing-walk: PASS')

assert_result()
