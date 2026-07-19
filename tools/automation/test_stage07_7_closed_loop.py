import argparse
import json
import math
import re
import sys
import time

import serial


STATUS_RE = re.compile(
    r"configured=(?P<configured>\d+) running=(?P<running>\d+) valid=(?P<valid>\d+) "
    r"mode=(?P<mode>\d+) state=(?P<state>\w+).*processed=(?P<processed>\d+) "
    r"rejected=(?P<rejected>\d+).*uncertainty=(?P<uncertainty>\d+) "
    r"nominal_ftw=0x(?P<nominal>[0-9A-Fa-f]+) ftw=0x(?P<ftw>[0-9A-Fa-f]+) "
    r"seq_initial=(?P<seq_initial>\d+) seq_current=(?P<seq_current>\d+).*"
    r"faults=(?P<faults>\d+)"
)
LOG_RE = re.compile(
    r"DPLL_CLOSED_LOOP.*state=(?P<state>\w+) error=(?P<error>[-+0-9.eE]+).*"
    r"uncertainty=(?P<uncertainty>\d+) ftw=0x(?P<ftw>[0-9A-Fa-f]+) "
    r"seq=(?P<seq>\d+)"
)


def read_line(ser, log):
    raw = ser.readline()
    if not raw:
        return None
    line = raw.decode("utf-8", errors="ignore").strip()
    if line:
        log.append(line)
        return line
    return None


def send(ser, request, prefix, timeout_s, log):
    ser.write((request + "\r\n").encode("ascii"))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = read_line(ser, log)
        if line and line.startswith(prefix):
            return line
    return None


def parse_status(line):
    match = STATUS_RE.search(line or "")
    if not match:
        return None
    item = match.groupdict()
    for key in ("configured", "running", "valid", "mode", "processed", "rejected",
                "uncertainty", "seq_initial", "seq_current", "faults"):
        item[key] = int(item[key])
    item["nominal"] = int(item["nominal"], 16)
    item["ftw"] = int(item["ftw"], 16)
    return item


def query_status(ser, log):
    return parse_status(send(ser, "CMD:DPLL_STATUS", "ACK:DPLL_STATUS", 2.0, log))


def wait_for_state(ser, wanted, timeout_s, log, seen_states, samples):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        status = query_status(ser, log)
        if status:
            seen_states.add(status["state"])
            samples.append(status)
            if status["state"] in wanted:
                return status
        time.sleep(0.05)
    return None


def collect_locked_logs(ser, duration_s, log):
    samples = []
    deadline = time.time() + duration_s
    while time.time() < deadline:
        line = read_line(ser, log)
        match = LOG_RE.search(line or "")
        if match and match.group("state") == "LOCKED":
            samples.append({
                "error": float(match.group("error")),
                "uncertainty": int(match.group("uncertainty")),
                "ftw": int(match.group("ftw"), 16),
                "seq": int(match.group("seq")),
            })
    return samples


def add_check(checks, name, ok, detail):
    checks.append({"name": name, "ok": bool(ok), "detail": detail})


def protocol_error_count(line):
    match = re.search(r"protocol_errors=(\d+) result=(-?\d+)", line or "")
    if not match or match.group(2) != "0":
        return None
    return int(match.group(1))


def run(port, baud):
    checks = []
    log = []
    status_samples = []
    seen_states = set()
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2))
        return 1

    with ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        initial_info = send(ser, "CMD:FPGA_INFO", "ACK:FPGA_INFO", 3.0, log)
        initial_protocol_errors = protocol_error_count(initial_info)
        add_check(checks, "initial_fpga_info", initial_protocol_errors is not None, initial_info)

        config = send(ser, "CMD:DPLL_CONFIG,30000,50000,100", "ACK:DPLL_CONFIG", 3.0, log)
        add_check(checks, "config", config and "result=0" in config, config)
        started = send(ser, "CMD:DPLL_CLOSED_LOOP_START", "ACK:DPLL_CLOSED_LOOP_START", 5.0, log)
        add_check(checks, "closed_loop_start",
                  started and "result=0" in started and "mode=RAW_FTW_PI" in started,
                  started)

        locked = wait_for_state(ser, {"LOCKED"}, 20.0, log, seen_states, status_samples)
        add_check(checks, "initial_lock", locked is not None,
                  f"seen={sorted(seen_states)} final={locked}")

        locked_logs = collect_locked_logs(ser, 3.0, log) if locked else []
        if locked_logs:
            rms_rad = math.sqrt(sum(x["error"] ** 2 for x in locked_logs) / len(locked_logs))
            peak_rad = max(abs(x["error"]) for x in locked_logs)
        else:
            rms_rad = float("inf")
            peak_rad = float("inf")
        add_check(checks, "locked_phase_quality",
                  len(locked_logs) >= 10 and rms_rad <= math.radians(5.0) and
                  peak_rad <= math.radians(10.0),
                  {"samples": len(locked_logs), "rms_deg": math.degrees(rms_rad),
                   "peak_deg": math.degrees(peak_rad)})

        fault = send(ser, "CMD:DPLL_FAULT_INJECT,60", "ACK:DPLL_FAULT_INJECT", 3.0, log)
        add_check(checks, "fault_inject", fault and "count=60" in fault and "result=0" in fault,
                  fault)
        holdover = wait_for_state(ser, {"HOLDOVER", "LOST"}, 2.0, log,
                                  seen_states, status_samples)
        lost = wait_for_state(ser, {"LOST"}, 3.0, log, seen_states, status_samples)
        add_check(checks, "holdover_seen", "HOLDOVER" in seen_states,
                  f"seen={sorted(seen_states)} first={holdover}")
        add_check(checks, "lost_seen", lost is not None, f"seen={sorted(seen_states)}")
        recovered = wait_for_state(ser, {"LOCKED"}, 20.0, log, seen_states, status_samples)
        add_check(checks, "recovered_lock", recovered is not None,
                  f"seen={sorted(seen_states)} final={recovered}")

        final_status = query_status(ser, log)
        status_samples.append(final_status) if final_status else None
        if final_status:
            correction_limit = final_status["nominal"] * 200.0e-6 + 2.0
            ftw_safe = all(
                abs(sample["ftw"] - sample["nominal"]) <= correction_limit
                for sample in status_samples if sample and sample["nominal"]
            )
            status_ok = (final_status["configured"] == 1 and final_status["running"] == 1 and
                         final_status["mode"] == 2 and final_status["state"] == "LOCKED" and
                         final_status["uncertainty"] <= 256 and
                         final_status["seq_current"] != final_status["seq_initial"])
        else:
            ftw_safe = False
            status_ok = False
        add_check(checks, "final_status", status_ok, final_status)
        add_check(checks, "ftw_bounded_200ppm", ftw_safe,
                  {"sample_count": len(status_samples), "final": final_status})

        info = send(ser, "CMD:FPGA_INFO", "ACK:FPGA_INFO", 3.0, log)
        final_protocol_errors = protocol_error_count(info)
        add_check(checks, "fpga_protocol_clean",
                  initial_protocol_errors is not None and
                  final_protocol_errors == initial_protocol_errors,
                  {"before": initial_protocol_errors, "after": final_protocol_errors,
                   "response": info})

        stopped = send(ser, "CMD:DPLL_OPEN_LOOP_STOP", "ACK:DPLL_OPEN_LOOP_STOP", 3.0, log)
        add_check(checks, "stop", stopped and "result=0" in stopped, stopped)

    passed = all(item["ok"] for item in checks)
    report = {
        "status": "PASS" if passed else "FAIL",
        "checks": checks,
        "seen_states": sorted(seen_states),
        "locked_log_samples": len(locked_logs),
        "log_tail": log[-80:],
    }
    print(json.dumps(report, indent=2, allow_nan=True))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    sys.exit(run(args.port, args.baud))
