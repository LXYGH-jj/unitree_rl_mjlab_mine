#!/usr/bin/env python3
"""M2.4d3 read-only binary analysis for vendor libZ1_SDK_x86_64.so.
NEVER dlopen/run/load the SDK, open sockets, or interact with any device.
"""
import argparse
import hashlib
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

TARGETS = (
    ("arm_sendrecv", "UNITREE_ARM::unitreeArm::sendRecv()"),
    ("ctrl_sendrecv", "UNITREE_ARM::CtrlComponents::sendRecv()"),
    ("udp_recv_default", "UNITREE_ARM::UDPPort::recv(unsigned char*)"),
    ("udp_recv_sized", "UNITREE_ARM::UDPPort::recv(unsigned char*, unsigned long)"),
    ("udp_recv_block", "UNITREE_ARM::UDPPort::_recvBlock("),
    ("udp_recv_unblock", "UNITREE_ARM::UDPPort::_recvUnBlock("),
    ("udp_send", "UNITREE_ARM::UDPPort::send(unsigned char*, unsigned long)"),
    ("arm_starttrack", "UNITREE_ARM::unitreeArm::startTrack("),
)
INTERESTING_LIBC = {"recvfrom", "recvmsg", "recvmmsg", "sendto", "sendmsg",
                    "select", "pselect", "poll", "epoll_wait", "socket",
                    "bind", "connect", "getsockname", "getpeername"}


def run(*args, timeout=20):
    try:
        return subprocess.run(args, check=True, capture_output=True, text=True,
                              timeout=timeout).stdout
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Command failed: {args[0]}: {e.stderr.strip()[:800]}") from e


def parse_symbols(s):
    result = []
    pat = re.compile(r"^([0-9a-fA-F]+)\s+([0-9a-fA-F]+)\s+([A-Za-z])\s+(.+)$")
    for line in s.splitlines():
        m = pat.match(line.strip())
        if m:
            addr, size, kind, name = m.groups()
            result.append(dict(address=int(addr, 16), size=int(size, 16),
                               kind=kind, name=name))
    return result


def choose(symbols, pattern):
    if pattern.endswith("("):
        arr = [s for s in symbols if s["name"].startswith(pattern)]
    else:
        arr = [s for s in symbols if s["name"] == pattern]
    arr = [s for s in arr if s["size"] > 0 and s["kind"].lower() in "twi"]
    # Check for aliases and prefer substantial canonical implementation.
    if not arr:
        return None
    arr.sort(key=lambda s: (-s["size"], s["address"]))
    return arr[0]


def disassemble(so, s):
    start = s["address"]
    # Cap is solely to keep a malformed ELF/symbol from creating huge reports.
    size = min(s["size"], 65536)
    return run("objdump", "-d", "-C", "-M", "intel",
               f"--start-address=0x{start:x}", f"--stop-address=0x{start+size:x}",
               str(so), timeout=40)


def calls_and_hotspots(asm):
    calls = []
    excerpts = []
    lines = asm.splitlines()
    for i, line in enumerate(lines):
        # Intel x86_64 disassembly, with optional plt target angle brackets.
        if re.search(r"\b(?:call|callq|jmp|jmpq)\s", line):
            m = re.search(r"<([^>]+)>", line)
            target = m.group(1) if m else line.strip().split("\t")[-1]
            calls.append(dict(address=line.strip().split(":")[0], target=target))
        if re.search(r"\b(?:recvfrom|recvmsg|recvmmsg|select|poll|sendto|sendmsg)(?:@|\+|>)", line):
            lo, hi = max(0, i-14), min(len(lines), i+8)
            excerpts.append("\n".join(lines[lo:hi]))
    return calls, excerpts


def check_headers(sdk, out):
    root = sdk.resolve().parent.parent
    files = {
      "unitreeArm": root / "include/unitree_arm_sdk/control/unitreeArm.h",
      "ctrlComponents": root / "include/unitree_arm_sdk/control/ctrlComponents.h",
      "UDPPort": root / "include/unitree_arm_sdk/message/udp.h",
      "arm_common": root / "include/unitree_arm_sdk/message/arm_common.h",
    }
    evidence = {}
    for key, path in files.items():
        if not path.is_file():
            evidence[key] = {"found": False, "path": str(path)}
            continue
        txt = path.read_text(encoding="utf-8", errors="replace")
        evidence[key] = {"found": True, "path": str(path), "sha256": hashlib.sha256(txt.encode()).hexdigest()}
        if key == "unitreeArm":
            evidence[key]["public_sdk_methods"] = {
                "sendRecv": "void sendRecv()" in txt,
                "startTrack": "void startTrack(ArmFSMState fsm)" in txt,
                "sendRecvThread": "LoopFunc *sendRecvThread" in txt,
                "lowstate_pointer": "LowlevelState *lowstate" in txt,
            }
        if key == "UDPPort":
            evidence[key]["recv_overloads"] = [line.strip() for line in txt.splitlines()
                                                   if re.search(r"\brecv\s*\(", line)]
        if key == "arm_common":
            mm = re.search(r"struct\s+RecvState\s*\{(.*?)\};", txt, re.S)
            evidence[key]["recvstate_body"] = mm.group(1).strip() if mm else "NOT FOUND"
            evidence[key]["has_declared_sequence_or_receive_timestamp"] = bool(
                mm and re.search(r"\b(?:sequence|seq|timestamp|time_stamp|rx_time)\b", mm.group(1), re.I))
    (out / "header_evidence.json").write_text(json.dumps(evidence, indent=2, ensure_ascii=False) + "\n")
    return evidence


def main():
    ap = argparse.ArgumentParser(description="Read-only Z1 SDK ELF call-path auditor; NEVER executes vendor code")
    ap.add_argument("--sdk", required=True, type=Path, help="Existing local libZ1_SDK_x86_64.so")
    ap.add_argument("--out", required=True, type=Path, help="Report directory (must not equal SDK directory)")
    ap.add_argument("--expect-sha256", default=None, help="Abort if SDK binary differs from expected known build")
    args = ap.parse_args()
    for tool in ("nm", "objdump", "readelf"):
        if not shutil.which(tool):
            ap.error(f"missing required read-only binutils: {tool}")
    so = args.sdk.resolve()
    if not so.is_file():
        ap.error(f"SDK ELF does not exist: {so}")
    if args.out.resolve() == so.parent or args.out.resolve() == so.parent.parent:
        ap.error("Do not place report in the SDK or lib directory")
    blob = so.read_bytes()  # read-only; never dlopen or execute
    sha = hashlib.sha256(blob).hexdigest()
    if args.expect_sha256 and sha.lower() != args.expect_sha256.lower():
        ap.error(f"SDK SHA mismatch: {sha} != {args.expect_sha256}")
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "disassembly").mkdir(exist_ok=True)
    info = run("readelf", "-h", str(so))
    if "X86-64" not in info:
        ap.error("Expected x86_64 ELF; no analysis performed")
    notes = run("readelf", "-n", str(so))
    symbols = parse_symbols(run("nm", "-S", "-C", "--defined-only", str(so), timeout=30))
    dyn = run("objdump", "-T", str(so))
    imported = sorted({l.split()[-1].split("@")[0]
                       for l in dyn.splitlines() if "*UND*" in l
                       and l.split()[-1].split("@")[0] in INTERESTING_LIBC})
    found = {}
    for label, pattern in TARGETS:
        s = choose(symbols, pattern)
        if not s:
            found[label] = {"present": False, "lookup": pattern}
            continue
        asm = disassemble(so, s)
        calls, excerpts = calls_and_hotspots(asm)
        fn = out / "disassembly" / f"{label}.asm"
        fn.write_text(asm)
        found[label] = {"present": True, "symbol": s["name"], "address": f"0x{s['address']:x}",
                        "size": s["size"], "calls": calls,
                        "syscall_adjacent_excerpts": excerpts,
                        "assembly_file": str(fn.relative_to(out))}
    header = check_headers(so, out)
    build_ids = re.findall(r"Build ID:\s*([0-9a-f]+)", notes)
    result = {"tool": "M2.4d3 READ-ONLY ELF ANALYSIS", "sdk": str(so), "sha256": sha,
              "build_ids": build_ids, "time_utc": datetime.now(timezone.utc).isoformat(),
              "imports": imported, "symbols": found, "header": header,
              "certifies_new_packet_proof": False,
              "certifies_atomic_state_copy": False,
              "certifies_peer_identity": False,
              "permits_vendor_command_or_robot": False}
    (out / "evidence.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    md = ["# M2.4d3 Z1 SDK STATIC RECEIVE-PATH AUDIT", "",
          "> READ ONLY: does not dlopen/start the SDK, create a socket, or send motor commands.",
          "> No binary reverse-analysis result alone establishes an independently proven fresh UDP feedback sample.",
          "", f"SDK SHA256: `{sha}`", f"ELF Build ID: `{', '.join(build_ids) or 'not found'}`",
          "", "## Imported UDP-related libc functions", ", ".join(imported) or "none detected",
          "", "## Selected SDK call graph (direct instructions, no runtime proof)"]
    for label, s in found.items():
        md.append("")
        if not s["present"]:
            md.append(f"### {label}: symbol not resolved (may be local/stripped/renamed)")
            continue
        md += [f"### {label} — `{s['symbol']}`", f"Address `{s['address']}`, length {s['size']} bytes",
               f"Disassembly: `{s['assembly_file']}`", "", "Calls/jumps:"]
        for call in s["calls"][:100]:
            md.append(f"- `{call['address']}` → `{call['target']}`")
        if len(s["calls"]) > 100:
            md.append(f"- ... {len(s['calls']) - 100} further calls in evidence.json")
        if not s["calls"]:
            md.append("- No direct calls resolved in chosen symbol interval")
        for n, excerpt in enumerate(s["syscall_adjacent_excerpts"][:6], start=1):
            md += ["", f"Syscall-adjacent snippet {n} (assembly only):", "```asm", excerpt, "```"]
    md += ["", "## Public header contract checks"]
    for key, h in header.items():
        md.append(f"- {key}: {'found' if h['found'] else 'MISSING'}")
    md += ["", "## Engineering decisions / unresolved proof", "",
           "1. `sendRecv()` is documented to BOTH send and receive. It is not a passive state sampler.",
           "2. Public `RecvState` does not advertise a packet sequence or receive timestamp; this audit does not supply one.",
           "3. A successful lowstate getter is not proof of new UDP input; disassembly/ELF symbols cannot repair that.",
           "4. Live receiver evidence must come from a reviewed receive/decode boundary, with receiver-origin timestamp, expected peer and same-datagram atomic state.",
           "5. The Z1 SDK receive thread (if used) must be the *only* sendRecv owner; do not add a competing call.",
           "6. Existing M2.4c2 pre-DDS real-output lock and unresolved electrical/fuse fault remain hard blocks.",
           "7. NEXT: review `disassembly/ctrl_sendrecv.asm`, `udp_recv_*.asm` and evidence.json before deciding whether vendor source/callback is required.",
           "", "IMPORTANT: Disassembly identifies calls only, NOT semantics of actual packets nor authentic sender identity."]
    (out / "REPORT.md").write_text("\n".join(md) + "\n")
    print("M2.4d3 SDK STATIC AUDIT: PASS (read-only analysis; NO SDK execution/IO)")
    print("SDK_SHA256=" + sha)
    print("BUILD_ID=" + (", ".join(build_ids) or "not found"))
    print("NETWORK_IMPORTS=" + (", ".join(imported) or "none detected"))
    for key, val in found.items():
        print(f"{key}: {'found '+val['address']+' size='+str(val['size']) if val['present'] else 'NOT FOUND'}")
    print("REPORT=" + str(out / "REPORT.md"))
    print("EVIDENCE=" + str(out / "evidence.json"))
    print("SAFETY: NO REAL UDP freshness proof; NO JOINTCTRL motor enable")

if __name__ == "__main__":
    main()
