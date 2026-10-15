#!/usr/bin/env python3
"""Read-only evidence checker for M2.4d6 MuJoCo walking + Z1 IK overlap.

No DDS, network access, process control, SDK, or actuator commands.
"""
import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path

STAMP = re.compile(r'^\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d+)\]')
WALK = re.compile(r'\[M20a\] W vx=([-+\d.eE]+) at sim tick=(\d+)ms')
STOP = re.compile(r'\[M20a\] max forward (\d+)ms reached; vx=0')
IK = re.compile(r'\[M20a\] IK TARGET XYZ_B=\[([^\]]+)\] fk_err=([-+\d.eE]+)')
DIAG = re.compile(r'\[M20diag\].*?\bvx=([-+\d.eE]+).*?\bphase=([-+\d.eE]+).*?\barm_track=([-+\d.eE]+).*?\btcp_err_m=([-+\d.eE]+).*?\barm_lock=(true|false)')
GATE = re.compile(r'\[M24d5\] SIM mock gate sent=(\d+) measured_q0=([-+\d.eE]+) gated_target_q0=([-+\d.eE]+)')
FATAL = re.compile(r'\b(?:SIM FAULT|SHADOW FAULT|FAULT latched|ARM MOTION LOCK|mock gate is disarmed|JOINTCTRL fault latched|M2\.4d5 mock integration limit|SIM LowState timeout)\b', re.I)


@dataclass
class Episode:
    start: str
    stop: str = ''
    duration_sec: float = 0.0
    forward_diagnostics: int = 0
    ik_targets_during_walk: int = 0
    mock_gate_samples_during_walk: int = 0
    gate_count_growth: int = 0
    q0_goal_range_during_walk_rad: float = 0.0
    peak_arm_tracking_rad: float = 0.0
    peak_tcp_error_m: float = 0.0
    observed_arm_motion_during_walk: bool = False
    evidence: str = ''
    notes: list = field(default_factory=list)


def time_of(line):
    match = STAMP.search(line)
    return datetime.fromisoformat(match.group(1)) if match else None


def analyze(text):
    state = False
    starts = []
    ends = []
    iks = []
    diags = []
    gates = []
    faults = []
    entered_count = 0
    for line_no, line in enumerate(text.splitlines(), start=1):
        tm = time_of(line)
        if 'FSM: Change state from ' in line and ' to JointCtrlMockSim' in line:
            state = True
            entered_count += 1
        elif 'FSM: Change state from JointCtrlMockSim to ' in line:
            state = False
        if not tm:
            continue
        if state and FATAL.search(line):
            faults.append({'line': line_no, 'message': line.strip()})
        if not state:
            continue
        m = WALK.search(line)
        if m:
            starts.append((tm, line_no, float(m.group(1))))
        if STOP.search(line):
            ends.append((tm, line_no))
        m = IK.search(line)
        if m:
            iks.append((tm, line_no, m.group(1), float(m.group(2))))
        m = DIAG.search(line)
        if m:
            diags.append((tm, line_no, float(m.group(1)), float(m.group(2)), float(m.group(3)), float(m.group(4)), m.group(5)))
        m = GATE.search(line)
        if m:
            gates.append((tm, line_no, int(m.group(1)), float(m.group(2)), float(m.group(3))))

    episodes = []
    used_stops = set()
    for t0, n0, vx in starts:
        matching = [(i, tm, ln) for i, (tm, ln) in enumerate(ends) if i not in used_stops and ln > n0 and tm > t0]
        if not matching:
            ep = Episode(start=t0.isoformat(sep=' '), evidence='INCOMPLETE', notes=['Missing corresponding max-forward auto-stop in captured log'])
            episodes.append(ep)
            continue
        idx, t1, n1 = matching[0]
        used_stops.add(idx)
        during = lambda ev: n0 < ev[1] < n1 and t0 <= ev[0] <= t1
        walk_ik = [ev for ev in iks if during(ev)]
        walk_diag = [ev for ev in diags if during(ev) and ev[2] > 0.01]
        walk_gate = [ev for ev in gates if during(ev)]
        ep = Episode(start=t0.isoformat(sep=' '), stop=t1.isoformat(sep=' '),
                     duration_sec=round((t1-t0).total_seconds(), 3),
                     forward_diagnostics=len(walk_diag), ik_targets_during_walk=len(walk_ik),
                     mock_gate_samples_during_walk=len(walk_gate))
        if walk_gate:
            ep.gate_count_growth = walk_gate[-1][2] - walk_gate[0][2]
            ep.q0_goal_range_during_walk_rad = round(max(x[4] for x in walk_gate)-min(x[4] for x in walk_gate), 5)
        if walk_diag:
            ep.peak_arm_tracking_rad = round(max(x[4] for x in walk_diag), 5)
            ep.peak_tcp_error_m = round(max(x[5] for x in walk_diag), 5)
        if vx <= 0 or not walk_ik or not walk_diag or len(walk_gate) < 2 or ep.gate_count_growth <= 0:
            ep.evidence = 'NO_CONFIRMED_OVERLAP'
            if not walk_ik: ep.notes.append('No accepted Z1 IK target inside active B2 walking interval')
            if not walk_diag: ep.notes.append('No positive-vx B2 diagnostics inside active walking interval')
            if len(walk_gate) < 2 or ep.gate_count_growth <= 0: ep.notes.append('Insufficient increasing Mock gate samples during walk')
        elif any(x[6] == 'true' for x in walk_diag):
            ep.evidence = 'SAFETY_LOCK'
            ep.notes.append('Arm motion lock occurred during walk')
        elif ep.q0_goal_range_during_walk_rad > 0.01:
            ep.evidence = 'OVERLAP_WITH_Q0_OUTPUT_CHANGE'
            ep.observed_arm_motion_during_walk = True
        else:
            ep.evidence = 'OVERLAP_COMMAND_PATH_ONLY'
            ep.notes.append('Accepted IK during positive-vx walk; 1Hz q0 telemetry does not prove changed Z1 motor target')
        episodes.append(ep)
    return {
        'tool': 'M2.4d6 offline log verification (MuJoCo-only)',
        'entered_mock_fsm': entered_count,
        'walk_episodes': [asdict(ep) for ep in episodes],
        'faults': faults,
        'overall': ('FAULT' if faults else
                    'PASS_OUTPUT_EVIDENCE' if any(e.evidence == 'OVERLAP_WITH_Q0_OUTPUT_CHANGE' for e in episodes) else
                    'PASS_COMMAND_PATH_ONLY' if any(e.evidence == 'OVERLAP_COMMAND_PATH_ONLY' for e in episodes) else
                    'INCOMPLETE' if any(e.evidence == 'INCOMPLETE' for e in episodes) else
                    'NO_SIMULTANEOUS_EVIDENCE'),
        'limitations': '1 Hz controller diagnostics cannot prove true end-effector tracking or hardware receive/actuation. Review MuJoCo video/pose and full 6-joint values separately.'
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path, help='Captured b2z1_ctrl console output (timestamps required)')
    parser.add_argument('--json-out', type=Path, help='Optional machine-readable report')
    args = parser.parse_args()
    if not args.log.is_file():
        parser.error('log does not exist: '+str(args.log))
    result = analyze(args.log.read_text(encoding='utf-8', errors='replace'))
    print('M2.4d6 RESULT:', result['overall'])
    print('  FSM entries:', result['entered_mock_fsm'], ' walking episodes:', len(result['walk_episodes']), ' faults:', len(result['faults']))
    for i, ep in enumerate(result['walk_episodes'], 1):
        print(f"  episode {i}: {ep['evidence']} walk={ep['duration_sec']:.3f}s IK-in-walk={ep['ik_targets_during_walk']} "+
              f"positive-vx-diag={ep['forward_diagnostics']} mock-gate={ep['mock_gate_samples_during_walk']} "+
              f"gate-growth={ep['gate_count_growth']} q0-range={ep['q0_goal_range_during_walk_rad']:.4f}rad")
        for note in ep['notes']: print('    NOTE:', note)
    for fault in result['faults'][:8]: print('  FAULT:', fault['message'])
    print('LIMITATION:', result['limitations'])
    if args.json_out:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
        print('Report saved:', args.json_out)
    return 0 if result['overall'] in ('PASS_OUTPUT_EVIDENCE', 'PASS_COMMAND_PATH_ONLY') else 2


if __name__ == '__main__':
    sys.exit(main())
