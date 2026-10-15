#!/usr/bin/env python3
"""Offline Z1 mock trace verification and CSV summary. NEVER starts physical SDK."""
import argparse
import csv
import math
import socket
import statistics
import subprocess
import tempfile
import time
from pathlib import Path


def _percentile(values, fraction):
    """Simple nearest-rank percentile; for diagnostics only."""
    seq = sorted(values)
    return seq[min(len(seq) - 1, math.ceil(fraction * len(seq)) - 1)]


def _span(values):
    return max(values) - min(values)


def _joint_summary(rows, label, all_six=False):
    if not rows:
        return
    print(f"{label} window: {len(rows)} samples, "
          f"{(int(rows[-1]['mono_ns'])-int(rows[0]['mono_ns']))/1e9:.3f} s")
    for j in (range(1, 7) if all_six else (3, 4, 5)):
        cmd = [float(r[f"qcmd_J{j}"]) for r in rows]
        goal = [float(r[f"goal_J{j}"]) for r in rows]
        meas = [float(r[f"qmeas_J{j}"]) for r in rows]
        dq = [float(r[f"dqmeas_J{j}"]) for r in rows]
        ff = [float(r[f"ff_sent_J{j}"]) for r in rows]
        feedback_tau = [float(r[f"tau_meas_J{j}"]) for r in rows]
        err = [m-c for m, c in zip(meas, cmd)]
        rms_err = math.sqrt(sum(v * v for v in err) / len(err))
        print(
            f"  J{j}: goal_span={_span(goal):.6f} "
            f"qcmd_span={_span(cmd):.6f} qmeas_span={_span(meas):.6f} rad "
            f"error_rms={rms_err:.6f} error_max={max(abs(v) for v in err):.6f} rad "
            f"max_abs_dq={max(abs(v) for v in dq):.5f} rad/s "
            f"ff_span={_span(ff):.4f} feedback_tau_span={_span(feedback_tau):.4f} Nm"
        )


def analyze(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError("Trace CSV has no rows")
    required = ["mono_ns", "cycle_dt_ms", "sdk_exchange_ms", "track", "target_seq"]
    for j in range(1, 7):
        for prefix in ("goal", "qcmd", "dqcmd", "qmeas", "dqmeas", "ff_sent", "tau_meas"):
            required.append(f"{prefix}_J{j}")
    absent = [k for k in required if k not in rows[0]]
    if absent:
        raise RuntimeError(f"Missing trace columns: {absent}")
    dt = [float(r["cycle_dt_ms"]) for r in rows]
    sdk_io = [float(r["sdk_exchange_ms"]) for r in rows]
    active = [r for r in rows if r["track"] == "1"]
    hold = [r for r in rows if r["track"] == "0"]
    print(f"CSV: {csv_path} | samples={len(rows)} | HOLD={len(hold)} | TRACK={len(active)}")
    print("Cycle dt (ms): min={:.3f} median={:.3f} p95={:.3f} p99={:.3f} max={:.3f}".format(
        min(dt), statistics.median(dt), _percentile(dt,.95), _percentile(dt,.99), max(dt)))
    print("SDK exchange (ms): median={:.3f} p95={:.3f} p99={:.3f} max={:.3f}".format(
        statistics.median(sdk_io), _percentile(sdk_io,.95), _percentile(sdk_io,.99), max(sdk_io)))
    if hold:
        # Only the last two seconds of the complete trace, not the last two
        # seconds of sparse HOLD samples mixed with TRACK intervals.
        last_ns = int(rows[-1]["mono_ns"])
        hold_tail = [r for r in hold if int(r["mono_ns"]) >= last_ns - 2_000_000_000]
        if hold_tail:
            _joint_summary(hold_tail, "Final HOLD (last 2s)", all_six=True)
        else:
            _joint_summary(hold[-min(100,len(hold)):], "Last recorded HOLD", all_six=True)
    if active:
        _joint_summary(active[-min(100,len(active)):], "Last 100 TRACK", all_six=False)
    print("Interpretation: constant qcmd with varying qmeas indicates feedback motion, "
          "not necessarily periodic oscillation; inspect the time series before diagnosis.")
    print("NOTE: SDK timestamps mark getter sampling, not confirmed fresh UDP packets.")
    return rows


def request(sock, packet):
    sock.sendall(packet.encode("ascii"))
    resp = sock.recv(512).decode("ascii")
    if not resp or resp.startswith("V1 ERR"):
        raise RuntimeError(f"Mock IPC rejected request {packet[:24]}: {resp}")
    return resp.split()


def mock_check(worker):
    if not worker.is_file():
        raise FileNotFoundError(f"Build Worker first: {worker}")
    folder = Path(tempfile.mkdtemp(prefix="z1_mock_trace_"))
    sock_path = folder / "worker.sock"
    csv_path = folder / "trace.csv"
    cmd = [str(worker), "--backend", "mock", "--socket", str(sock_path),
           "--trace-csv", str(csv_path)]
    print("Starting MOCK-ONLY worker (no motor SDK, no robot):", " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True)
    try:
        until = time.monotonic() + 5
        while not sock_path.exists() and time.monotonic() < until:
            if proc.poll() is not None:
                raise RuntimeError("Mock Worker terminated before socket ready")
            time.sleep(0.01)
        with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as conn:
            conn.settimeout(2.)
            conn.connect(str(sock_path))
            hello = request(conn, "V1 HELLO")
            if "MOCK_ONLY" not in hello:
                raise RuntimeError(f"Refusing non-mock Worker: {hello}")
            raw = request(conn, "V1 GET 1")
            if len(raw) != 20 or raw[2] != "MOCK":
                raise RuntimeError(f"Unexpected mock STATE: {raw}")
            start = [float(x) for x in raw[8:14]]
            # REF must expose the exact hold COMMAND; GET is motor/mock
            # measurement, which may differ in real SDK mode.
            ref_packet = request(conn, "V1 REF 2")
            if (len(ref_packet) != 11 or ref_packet[:3] != ["V1", "REF", "2"]
                    or ref_packet[4] != "HOLD"):
                raise RuntimeError(f"Malformed initial mock REF: {ref_packet}")
            ref_pose = [float(v) for v in ref_packet[5:11]]
            if any(abs(x-y)>1e-8 for x,y in zip(ref_pose,start)):
                raise RuntimeError("Mock HOLD reference mismatches starting command")
            target = list(ref_pose)
            target[4] += 0.005   # J5 mock-only setpoint
            if not all(math.isfinite(v) for v in target):
                raise RuntimeError("Bad mock starting pose")
            for seq in range(1, 201):
                goal = target if seq <= 100 else start
                values = " ".join(f"{v:.10f}" for v in goal)
                msg = f"V1 TARGET {seq} {time.monotonic_ns()} 1 {values}"
                ack = request(conn, msg)
                if ack[:3] != ["V1", "ACK", str(seq)]:
                    raise RuntimeError(f"Unexpected target ack: {ack}")
                if seq in (1, 100, 101, 200):
                    ref_now = request(conn, f"V1 REF {seq + 2}")
                    if len(ref_now) != 11 or ref_now[4] not in ("HOLD", "TRACK"):
                        raise RuntimeError(f"Malformed tracking REF: {ref_now}")
                time.sleep(0.02)
        until = time.monotonic() + 3
        previous_size = -1
        stable_checks = 0
        while time.monotonic() < until:
            size = csv_path.stat().st_size if csv_path.exists() else 0
            stable_checks = stable_checks + 1 if size > 0 and size == previous_size else 0
            if stable_checks >= 3:
                break
            previous_size = size
            time.sleep(0.05)
        if stable_checks < 3:
            raise RuntimeError("Mock session did not finish writing trace CSV")
        rows = analyze(csv_path)
        active = [r for r in rows if r["track"] == "1"]
        if len(active) < 10:
            raise RuntimeError("Insufficient TRACK samples in mock run")
        final_cmd = float(active[-1]["qcmd_J5"])
        if abs(final_cmd - start[4]) > 1e-3:
            raise RuntimeError(f"Mock Planner did not return: {final_cmd} vs {start[4]}")
        if max(float(r["qcmd_J5"]) for r in active) < start[4] + 0.004:
            raise RuntimeError("Mock Planner never reached positive J5 target")
        print("MOCK PASS: J5 moved to +0.005 rad, returned to start, and CSV captured.")
        print("No physical motor behavior was tested.")
    finally:
        proc.terminate()
        try:
            stdout, _ = proc.communicate(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            stdout, _ = proc.communicate(timeout=3)
        if proc.returncode and "MOCK PASS" not in stdout:
            print("Worker process output (tail):", stdout[-1000:])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--analyze", type=Path, help="Analyze existing Z1 trace CSV")
    parser.add_argument("--worker", type=Path,
                        default=Path(__file__).parent / "build/m24e2d_z1_worker",
                        help="Worker executable (always launched with --backend mock)")
    args = parser.parse_args()
    if args.analyze:
        analyze(args.analyze)
    else:
        mock_check(args.worker.resolve())


if __name__ == "__main__":
    main()
