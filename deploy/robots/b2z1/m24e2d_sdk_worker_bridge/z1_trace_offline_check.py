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


def analyze(csv_path):
    with open(csv_path, newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        raise RuntimeError("Trace CSV has no rows")
    for key in ("mono_ns", "cycle_dt_ms", "sdk_exchange_ms", "track",
                "goal_J5", "qcmd_J5", "qmeas_J5", "dqmeas_J5",
                "ff_sent_J5", "tau_meas_J5"):
        if key not in rows[0]:
            raise RuntimeError(f"Missing trace column: {key}")
    dt = [float(r["cycle_dt_ms"]) for r in rows]
    sdk_io = [float(r["sdk_exchange_ms"]) for r in rows]
    active = [r for r in rows if r["track"] == "1"]
    print(f"CSV: {csv_path} | samples={len(rows)} | TRACK={len(active)}")
    print("Cycle dt (ms): min={:.3f} median={:.3f} max={:.3f}".format(
        min(dt), statistics.median(dt), max(dt)))
    print("SDK exchange (ms): median={:.3f} max={:.3f}".format(
        statistics.median(sdk_io), max(sdk_io)))
    if active:
        last = active[-min(100, len(active)):]
        for j in (3, 4, 5):
            goal = [float(r[f"goal_J{j}"]) for r in last]
            cmd = [float(r[f"qcmd_J{j}"]) for r in last]
            meas = [float(r[f"qmeas_J{j}"]) for r in last]
            speed = [float(r[f"dqmeas_J{j}"]) for r in last]
            print(f"J{j} last {len(last)} TRACK samples: "
                  f"goal_span={max(goal)-min(goal):.7f}, "
                  f"cmd_span={max(cmd)-min(cmd):.7f}, "
                  f"meas_span={max(meas)-min(meas):.7f} rad, "
                  f"max_abs_dq_meas={max(abs(v) for v in speed):.5f} rad/s")
    print("NOTE: An SDK CSV contains getter samples, not confirmed fresh UDP timestamps.")
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
            target = list(start)
            target[4] += 0.005   # J5 mock-only setpoint
            if not all(math.isfinite(v) for v in target):
                raise RuntimeError("Bad mock starting pose")
            values = " ".join(f"{v:.10f}" for v in target)
            for seq in range(1, 101):
                msg = f"V1 TARGET {seq} {time.monotonic_ns()} 1 {values}"
                ack = request(conn, msg)
                if ack[:3] != ["V1", "ACK", str(seq)]:
                    raise RuntimeError(f"Unexpected target ack: {ack}")
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
        if abs(final_cmd - target[4]) > 1e-3:
            raise RuntimeError(f"Mock Planner did not settle: {final_cmd} vs {target[4]}")
        print("MOCK PASS: repeated fixed target converged; CSV capture complete.")
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
