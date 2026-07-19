import argparse
import json
import re
import sys
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


def run(port, baud, count):
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

        pong = command(ser, "CMD:PING", "ACK:PONG", 3.0, log)
        checks.append({"name": "ping", "ok": pong == "ACK:PONG", "detail": pong})

        info = command(ser, "CMD:FPGA_INFO", "ACK:FPGA_INFO", 3.0, log)
        info_ok = bool(
            info
            and "id=0x2023" in info
            and "protocol=0x0002" in info
            and "capabilities=0x001F" in info
            and "build=0x0721" in info
            and "result=0" in info
        )
        checks.append({"name": "fpga_info", "ok": info_ok, "detail": info})
        initial_error_match = re.search(r"protocol_errors=(\d+)", info or "")
        initial_protocol_errors = int(initial_error_match.group(1)) if initial_error_match else None

        sine = command(ser, "CMD:FPGA_WAVE_BOTH,SINE", "ACK:FPGA_WAVE_BOTH", 3.0, log)
        triangle = command(ser, "CMD:FPGA_WAVE_BOTH,TRIANGLE", "ACK:FPGA_WAVE_BOTH", 3.0, log)
        sine_restore = command(ser, "CMD:FPGA_WAVE_BOTH,SINE", "ACK:FPGA_WAVE_BOTH", 3.0, log)
        wave_ok = bool(
            sine and "wave=SINE result=0" in sine
            and triangle and "wave=TRIANGLE result=0" in triangle
            and sine_restore and "wave=SINE result=0" in sine_restore
        )
        checks.append({"name": "wave_both_roundtrip", "ok": wave_ok,
                       "detail": [sine, triangle, sine_restore]})

        snapshot = command(ser, "CMD:FPGA_SNAPSHOT", "ACK:FPGA_SNAPSHOT", 3.0, log)
        snap_match = re.search(
            r"seq=(\d+) anchor=(\d+) uncertainty=(\d+) sample=0x([0-9A-F]+) "
            r"apply=0x([0-9A-F]+) phase_a=0x([0-9A-F]+) "
            r"phase_b=0x([0-9A-F]+) ftw_a=0x([0-9A-F]+) ftw_b=0x([0-9A-F]+) "
            r"status=0x([0-9A-F]+) result=(-?\d+)",
            snapshot or "",
        )
        snapshot_ok = bool(
            snap_match
            and int(snap_match.group(2)) > 0
            and int(snap_match.group(3)) <= 256
            and int(snap_match.group(4), 16) > 0
            and (int(snap_match.group(10), 16) & 0x2) != 0
            and snap_match.group(11) == "0"
        )
        checks.append({"name": "snapshot", "ok": snapshot_ok, "detail": snapshot})

        self_test = command(
            ser,
            f"CMD:FPGA_PROTOCOL_SELF_TEST,{count}",
            "ACK:FPGA_PROTOCOL_SELF_TEST",
            30.0,
            log,
        )
        test_match = re.search(
            r"count=(\d+) snapshot_errors=(\d+) seq_errors=(\d+) counter_errors=(\d+) "
            r"phase_errors=(\d+) status_errors=(\d+) raw_errors=(\d+) pass=(\d+)",
            self_test or "",
        )
        self_test_ok = bool(
            test_match
            and int(test_match.group(1)) == count
            and all(test_match.group(index) == "0" for index in range(2, 8))
            and test_match.group(8) == "1"
        )
        checks.append({"name": "protocol_self_test", "ok": self_test_ok, "detail": self_test})

        ll_status = command(ser, "CMD:SPI_LL_STATUS", "ACK:SPI_LL_STATUS", 3.0, log)
        ll_ok = bool(ll_status and "timeouts=0 errors=0 pass=1" in ll_status)
        checks.append({"name": "ll_status", "ok": ll_ok, "detail": ll_status})

        final_info = command(ser, "CMD:FPGA_INFO", "ACK:FPGA_INFO", 3.0, log)
        final_error_match = re.search(r"protocol_errors=(\d+)", final_info or "")
        final_protocol_errors = int(final_error_match.group(1)) if final_error_match else None
        checks.append({"name": "protocol_errors_stable",
                       "ok": initial_protocol_errors is not None and
                             final_protocol_errors == initial_protocol_errors,
                       "detail": {"before": initial_protocol_errors,
                                  "after": final_protocol_errors}})

    passed = all(check["ok"] for check in checks)
    print(json.dumps({"status": "PASS" if passed else "FAIL", "checks": checks,
                      "log_tail": log[-40:]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=1000)
    args = parser.parse_args()
    sys.exit(run(args.port, args.baud, args.count))
