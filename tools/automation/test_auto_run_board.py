import argparse
import json
import re
import time

import serial


def send(ser, request):
    ser.write((request + "\r\n").encode("ascii"))


def wait_prefix(ser, prefix, timeout_s, log):
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


def run(port, baud, phase_degrees, timeout_s):
    log = []
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, indent=2))
        return 1

    final = None
    with ser:
        time.sleep(0.5)
        ser.reset_input_buffer()
        send(ser, "CMD:AUTO_RUN_STOP")
        wait_prefix(ser, "ACK:AUTO_RUN_STOP", 2.0, log)
        send(ser, f"CMD:AUTO_RUN_START,{phase_degrees}")
        start_ack = wait_prefix(ser, "ACK:AUTO_RUN_START", 3.0, log)
        if not start_ack or "result=0" not in start_ack:
            print(json.dumps({"status": "FAIL", "start_ack": start_ack,
                              "log_tail": log[-30:]}, indent=2))
            return 1

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            send(ser, "CMD:AUTO_RUN_STATUS")
            line = wait_prefix(ser, "ACK:AUTO_RUN_STATUS", 2.0, log)
            if line:
                final = line
                match = re.search(r"state=([A-Z_]+).*result=(-?\d+)", line)
                if match and match.group(1) in {"LOCKED", "FAILED"}:
                    break
            time.sleep(0.5)

    passed = bool(final and "state=LOCKED" in final and "result=0" in final)
    print(json.dumps({"status": "PASS" if passed else "FAIL", "final": final,
                      "log_tail": log[-40:]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--phase", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=22.0)
    args = parser.parse_args()
    raise SystemExit(run(args.port, args.baud, args.phase, args.timeout))
