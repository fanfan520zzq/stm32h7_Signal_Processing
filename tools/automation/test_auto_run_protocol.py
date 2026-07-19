import argparse
import json
import time

import serial


def command(ser, request, prefix, timeout_s, log):
    ser.write((request + "\r\n").encode("ascii"))
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


def run(port, baud, quiet_seconds):
    checks = []
    log = []
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2))
        return 1

    with ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        stop = command(ser, "CMD:AUTO_RUN_STOP", "ACK:AUTO_RUN_STOP", 3.0, log)
        checks.append({"name": "stop", "ok": stop == "ACK:AUTO_RUN_STOP result=0",
                       "detail": stop})

        ser.reset_input_buffer()
        quiet_lines = []
        deadline = time.time() + quiet_seconds
        while time.time() < deadline:
            raw = ser.readline()
            if raw:
                line = raw.decode("utf-8", errors="ignore").strip()
                if line:
                    quiet_lines.append(line)
        checks.append({"name": "idle_uart_quiet", "ok": len(quiet_lines) == 0,
                       "detail": quiet_lines})

        pong = command(ser, "CMD:PING", "ACK:PONG", 3.0, log)
        checks.append({"name": "ping_single_ack", "ok": pong == "ACK:PONG",
                       "detail": pong})

        invalid = command(ser, "CMD:AUTO_RUN_START,3", "ACK:AUTO_RUN_START", 3.0, log)
        checks.append({"name": "phase_step_guard",
                       "ok": bool(invalid and "phase_deg=3 result=-1" in invalid),
                       "detail": invalid})

        status = command(ser, "CMD:AUTO_RUN_STATUS", "ACK:AUTO_RUN_STATUS", 3.0, log)
        checks.append({"name": "status_query",
                       "ok": bool(status and "state=IDLE" in status and "result=0" in status),
                       "detail": status})

    passed = all(item["ok"] for item in checks)
    print(json.dumps({"status": "PASS" if passed else "FAIL", "checks": checks,
                      "log_tail": log[-20:]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--quiet-seconds", type=float, default=3.0)
    args = parser.parse_args()
    raise SystemExit(run(args.port, args.baud, args.quiet_seconds))
