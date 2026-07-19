import argparse
import json
import re
import sys
import time

import serial


def read_line(ser, log):
    raw = ser.readline()
    if not raw:
        return None
    line = raw.decode("utf-8", errors="ignore").strip()
    if line:
        log.append(line)
    return line or None


def send(ser, request, prefix, timeout_s, log):
    ser.write((request + "\r\n").encode("ascii"))
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        line = read_line(ser, log)
        if line and line.startswith(prefix):
            return line
    return None


def fields(line):
    result = {}
    for key, value in re.findall(r"(\w+)=([^ ]+)", line or ""):
        result[key] = value
    return result


def as_int(value):
    return int(value, 16) if value.lower().startswith("0x") else int(value)


def protocol_errors(line):
    value = fields(line).get("protocol_errors")
    return int(value) if value is not None else None


def wait_locked(ser, timeout_s, log):
    deadline = time.time() + timeout_s
    last = None
    while time.time() < deadline:
        last = send(ser, "CMD:DPLL_STATUS", "ACK:DPLL_STATUS", 2.0, log)
        item = fields(last)
        if item.get("state") == "LOCKED" and item.get("running") == "1":
            return last
        time.sleep(0.05)
    return last


def check(checks, name, ok, detail):
    checks.append({"name": name, "ok": bool(ok), "detail": detail})


def run(port, baud):
    checks, log = [], []
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2))
        return 1

    with ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        initial_info = send(ser, "CMD:FPGA_INFO", "ACK:FPGA_INFO", 3.0, log)
        initial_errors = protocol_errors(initial_info)
        check(checks, "initial_info", initial_errors is not None, initial_info)

        config = send(ser, "CMD:DPLL_CONFIG,30000,60000,100", "ACK:DPLL_CONFIG", 3.0, log)
        check(checks, "derived_config", config and "result=0" in config, config)
        sweep_ok = True
        for phase in range(0, 181, 5):
            response = send(ser, f"CMD:DPLL_B_DERIVED,2,{phase}",
                            "ACK:DPLL_B_DERIVED", 2.0, log)
            sweep_ok &= bool(response and "result=0" in response)
        check(checks, "phase_sweep_0_180_step_5", sweep_ok, "37 commands")
        invalid_phase = send(ser, "CMD:DPLL_B_DERIVED,2,3", "ACK:DPLL_B_DERIVED", 2.0, log)
        invalid_ratio = send(ser, "CMD:DPLL_B_DERIVED,3,90", "ACK:DPLL_B_DERIVED", 2.0, log)
        check(checks, "invalid_phase_rejected", invalid_phase and "result=0" not in invalid_phase,
              invalid_phase)
        check(checks, "invalid_ratio_rejected", invalid_ratio and "result=0" not in invalid_ratio,
              invalid_ratio)
        selected = send(ser, "CMD:DPLL_B_DERIVED,2,90", "ACK:DPLL_B_DERIVED", 2.0, log)
        check(checks, "derived_select", selected and "result=0" in selected, selected)
        started = send(ser, "CMD:DPLL_CLOSED_LOOP_START", "ACK:DPLL_CLOSED_LOOP_START", 5.0, log)
        check(checks, "derived_start", started and "result=0" in started, started)
        derived_status_line = wait_locked(ser, 20.0, log)
        derived = fields(derived_status_line)
        derived_ftw_ok = bool(derived and as_int(derived["ftw_b"]) ==
                              ((2 * as_int(derived["ftw"])) & 0xFFFFFFFF))
        check(checks, "derived_locked", derived.get("state") == "LOCKED" and
              derived.get("b_mode") == "1" and derived.get("b_ratio") == "2" and
              derived.get("b_phase_deg") == "90", derived_status_line)
        check(checks, "derived_ftw_relation", derived_ftw_ok, derived_status_line)
        snapshot_line = send(ser, "CMD:FPGA_SNAPSHOT", "ACK:FPGA_SNAPSHOT", 3.0, log)
        snap = fields(snapshot_line)
        phase_relation = None
        if "phase_a" in snap and "phase_b" in snap:
            phase_relation = (as_int(snap["phase_b"]) - 2 * as_int(snap["phase_a"])) & 0xFFFFFFFF
        check(checks, "derived_phase_90", phase_relation == 0x40000000,
              {"relation": None if phase_relation is None else f"0x{phase_relation:08X}",
               "snapshot": snapshot_line})
        send(ser, "CMD:DPLL_OPEN_LOOP_STOP", "ACK:DPLL_OPEN_LOOP_STOP", 3.0, log)

        config = send(ser, "CMD:DPLL_CONFIG,30000,50000,100", "ACK:DPLL_CONFIG", 3.0, log)
        common = send(ser, "CMD:DPLL_B_COMMON", "ACK:DPLL_B_COMMON", 3.0, log)
        started = send(ser, "CMD:DPLL_CLOSED_LOOP_START", "ACK:DPLL_CLOSED_LOOP_START", 5.0, log)
        check(checks, "common_start", config and common and started and
              all("result=0" in item for item in (config, common, started)),
              {"config": config, "mode": common, "start": started})
        common_status_line = wait_locked(ser, 20.0, log)
        common_status = fields(common_status_line)
        common_relation = False
        if common_status:
            nominal_a = as_int(common_status["nominal_ftw"])
            nominal_b = as_int(common_status["nominal_ftw_b"])
            delta_a = as_int(common_status["ftw"]) - nominal_a
            expected_b = nominal_b + round(delta_a * nominal_b / nominal_a)
            common_relation = abs(as_int(common_status["ftw_b"]) - expected_b) <= 1
        check(checks, "common_ppm_locked", common_status.get("state") == "LOCKED" and
              common_status.get("b_mode") == "0" and common_relation, common_status_line)

        final_info = send(ser, "CMD:FPGA_INFO", "ACK:FPGA_INFO", 3.0, log)
        final_errors = protocol_errors(final_info)
        check(checks, "protocol_errors_stable", final_errors == initial_errors,
              {"before": initial_errors, "after": final_errors})
        stopped = send(ser, "CMD:DPLL_OPEN_LOOP_STOP", "ACK:DPLL_OPEN_LOOP_STOP", 3.0, log)
        check(checks, "stop", stopped and "result=0" in stopped, stopped)

    passed = all(item["ok"] for item in checks)
    print(json.dumps({"status": "PASS" if passed else "FAIL", "checks": checks,
                      "log_tail": log[-80:]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    sys.exit(run(args.port, args.baud))
