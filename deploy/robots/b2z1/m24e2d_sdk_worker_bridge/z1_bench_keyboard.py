#!/usr/bin/env python3
"""M2.4e2j: Z1-only manual bench control via the existing V1 Unix IPC protocol.

Never invokes B2, MuJoCo, or the SDK directly. The Worker owns the Z1 SDK.
Requires a mechanically secured arm, accessible physical E-stop and an operator.
"""
import argparse
import math
import os
import select
import socket
import sys
import termios
import time
import tty

STEP_RAD = 0.005
BENCH_MAX_DELTA_RAD = 0.03  # Per joint relative to the measured startup posture.
PERIOD = 0.02


def request(sock, msg):
    sock.sendall(msg.encode("ascii"))
    payload = sock.recv(512)
    if not payload:
        raise RuntimeError("Worker closed IPC connection")
    ans = payload.decode("ascii", errors="replace")
    if ans.startswith("V1 ERR"):
        raise RuntimeError(ans)
    return ans.split()


def state(sock, req):
    data = request(sock, f"V1 GET {req}")
    if len(data) != 20 or data[:2] != ["V1", "STATE"] or data[3] != str(req):
        raise RuntimeError(f"Malformed feedback: {data}")
    if data[2] != "SDK":
        raise RuntimeError("Refusing non-SDK Worker")
    q = [float(x) for x in data[8:14]]
    if not all(math.isfinite(x) for x in q):
        raise RuntimeError("Nonfinite Z1 joint feedback")
    return q, data


def target(sock, seq, q, enable):
    stamp = time.monotonic_ns()
    args = " ".join(f"{x:.10f}" for x in q)
    msg = f"V1 TARGET {seq} {stamp} {int(enable)} {args}"
    ans = request(sock, msg)
    if ans[:3] != ["V1", "ACK", str(seq)]:
        raise RuntimeError(f"Unexpected target response: {ans}")


def keypress(timeout):
    ready, _, _ = select.select([sys.stdin], [], [], timeout)
    if not ready:
        return ""
    return os.read(sys.stdin.fileno(), 1).decode("ascii", errors="ignore")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--socket", default="/tmp/b2z1_z1_worker.sock")
    args = ap.parse_args()
    if not sys.stdin.isatty():
        raise SystemExit("Interactive terminal required")
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as sock:
        # The Worker constructs its SDK state before answering the first HELLO.
        # Initial FSM mode transitions can take several seconds.
        sock.settimeout(30.0)
        sock.connect(args.socket)
        hello = request(sock, "V1 HELLO")
        if hello[:2] != ["V1", "HELLO_ACK"] or "SDK_MODE_UNQUALIFIED" not in hello:
            raise RuntimeError(f"Wrong Worker backend: {hello}")
        q, _ = state(sock, 1)
        sock.settimeout(0.25)  # Normal loop: fail promptly on lost IPC.
        origin = list(q)  # Return-to-start means this measured posture, NOT factory home.
        reference = list(q)
        req, seq, joint, armed = 1, 0, 0, False
        sent_active_target = False
        print("M2.4e2j Z1 bench keyboard. Physical support and E-stop REQUIRED.")
        print("1-6 select joint; a ARM; +/- jog 0.005rad; r RETURN to start; s STATUS; d DISARM; q QUIT.")
        print("'r' requires ARM; returns at Worker planner limits, never auto-homes on exit.")
        print("Before 'a', client only requests feedback: it sends NO TARGET packets.")
        print("No movement commands until explicit a. Keep tool/operator clear.")
        print("Initial q:", " ".join(f"{v:+.3f}" for v in q), flush=True)
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                tick = time.monotonic()
                key = keypress(PERIOD)
                req += 1
                current, _ = state(sock, req)
                if key in "123456" and key:
                    joint = int(key)-1
                    print(f"\nSelected joint {joint+1}", flush=True)
                elif key == "a" and not armed:
                    reference = list(current)
                    armed = True
                    print("\nARMED at measured joint positions", flush=True)
                elif key == "r" and armed:
                    reference = list(origin)
                    print("\nReturning toward startup joint posture; inspect collision path.", flush=True)
                elif key == "s":
                    qd = [float(x) for x in _[14:20]]
                    if not all(math.isfinite(x) for x in qd):
                        raise RuntimeError("Nonfinite Z1 velocity feedback")
                    sensed = request(sock, f"V1 SENSE {req}")
                    if len(sensed) != 16 or sensed[:2] != ["V1", "SENSE"] or sensed[2] != str(req):
                        raise RuntimeError(f"Invalid Z1 torque telemetry: {sensed}")
                    sensed_tau = [float(v) for v in sensed[4:10]]
                    temperatures = [int(v) for v in sensed[10:16]]
                    if not all(math.isfinite(v) for v in sensed_tau):
                        raise RuntimeError("Nonfinite Z1 torque telemetry")
                    deviation = max(abs(x-y) for x, y in zip(current, origin))
                    speed = max(abs(v) for v in qd)
                    print(f"\nq={current} max_start_error={deviation:.4f}rad"
                          f" max_speed={speed:.4f}rad/s"
                          f"\nfeedback_tau={sensed_tau} temperature_C={temperatures}",
                          flush=True)
                elif key == "d":
                    armed = False
                    reference = list(current)
                    print("\nDISARMED", flush=True)
                elif key in ("+", "=" , "-") and armed:
                    candidate = reference[joint] + (STEP_RAD if key in ("+", "=") else -STEP_RAD)
                    if abs(candidate - origin[joint]) > BENCH_MAX_DELTA_RAD + 1e-9:
                        print("\nBench joint travel envelope reached; command ignored", flush=True)
                    else:
                        reference[joint] = candidate
                        print(f"\nJoint {joint+1} target {reference[joint]:+.4f}", flush=True)
                elif key == "q":
                    break
                # Do not transmit any TARGET until the operator explicitly arms.
                # Once armed, maintain the existing <=150ms target watchdog.
                # On disarm, send exactly one enable=0 target.
                if armed:
                    seq += 1
                    target(sock, seq, reference, True)
                    sent_active_target = True
                elif sent_active_target:
                    seq += 1
                    target(sock, seq, current, False)
                    sent_active_target = False
                # Do not silently suppress link or target errors.
                remaining = PERIOD - (time.monotonic()-tick)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)
            # When no joint command was ever armed, closing the socket is enough
            # to end this SDK session; do not create a new TARGET on shutdown.
            if sent_active_target:
                try:
                    req += 1
                    current, _ = state(sock, req)
                    seq += 1
                    target(sock, seq, current, False)
                except (OSError, ValueError, RuntimeError):
                    pass
            print("\nKeyboard client stopped. Verify Worker PASSIVE confirmation.", flush=True)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as e:
        print(f"ERROR: {e}; use physical E-stop if arm is not controlled", file=sys.stderr)
        raise SystemExit(1)
