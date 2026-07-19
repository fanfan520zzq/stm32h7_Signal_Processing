import argparse
import json
import re
import sys
import time

import serial


def wait_for(ser, prefix, timeout_s, log):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue
        log.append(line)
        if line.startswith(prefix):
            return line
    return None


def send(ser, request, prefix, timeout_s, log):
    ser.write((request + "\r\n").encode("ascii"))
    return wait_for(ser, prefix, timeout_s, log)


def status_seq(line):
    match = re.search(r"seq=(\d+)\(", line or "")
    return int(match.group(1)) if match else None


def run(port, baud):
    log = []
    checks = []
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2))
        return 1

    with ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        config = send(ser, "CMD:DPLL_CONFIG,30000,50000,10", "ACK:DPLL_CONFIG", 3.0, log)
        checks.append({"name": "config", "ok": bool(config and "result=0" in config),
                       "detail": config})
        started = send(ser, "CMD:DPLL_OPEN_LOOP_START", "ACK:DPLL_OPEN_LOOP_START", 3.0, log)
        checks.append({"name": "start", "ok": bool(started and "result=0" in started and
                                                       "mode=OBSERVE_ONLY" in started),
                       "detail": started})

        initial_fpga = send(ser, "CMD:FPGA_STATUS", "ACK:FPGA_STATUS", 3.0, log)
        initial_seq = status_seq(initial_fpga)
        checks.append({"name": "initial_seq", "ok": initial_seq is not None,
                       "detail": initial_fpga})

        observations = []
        deadline = time.time() + 5.0
        while time.time() < deadline and len(observations) < 5:
            line = wait_for(ser, "LOG:INFO DPLL_OPEN_LOOP", 0.6, log)
            if line:
                observations.append(line)
        observation_ok = len(observations) >= 5 and all(
            "uncertainty=" in line and "ftw=0x" in line for line in observations
        )
        checks.append({"name": "observations", "ok": observation_ok,
                       "detail": f"count={len(observations)}"})

        dpll_status = send(ser, "CMD:DPLL_STATUS", "ACK:DPLL_STATUS", 3.0, log)
        status_match = re.search(
            r"configured=1 running=1 valid=1 processed=(\d+) rejected=(\d+).*"
            r"seq_initial=(\d+) seq_current=(\d+) write_free=(\d+)",
            dpll_status or "",
        )
        status_ok = bool(
            status_match
            and int(status_match.group(1)) >= 5
            and status_match.group(3) == status_match.group(4)
            and status_match.group(5) == "1"
        )
        checks.append({"name": "dpll_status", "ok": status_ok, "detail": dpll_status})

        final_fpga = send(ser, "CMD:FPGA_STATUS", "ACK:FPGA_STATUS", 3.0, log)
        final_seq = status_seq(final_fpga)
        checks.append({"name": "no_commit", "ok": initial_seq is not None and final_seq == initial_seq,
                       "detail": f"seq {initial_seq} -> {final_seq}"})

        stopped = send(ser, "CMD:DPLL_OPEN_LOOP_STOP", "ACK:DPLL_OPEN_LOOP_STOP", 3.0, log)
        checks.append({"name": "stop", "ok": bool(stopped and "result=0" in stopped),
                       "detail": stopped})

    passed = all(check["ok"] for check in checks)
    print(json.dumps({"status": "PASS" if passed else "FAIL", "checks": checks,
                      "observations": observations, "log_tail": log[-50:]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    sys.exit(run(args.port, args.baud))
