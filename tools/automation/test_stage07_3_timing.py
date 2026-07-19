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


def run(port, baud):
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

        ser.write(b"CMD:PING\r\n")
        pong = wait_for(ser, "ACK:PONG", 3.0, log)
        checks.append({"name": "ping", "ok": pong == "ACK:PONG", "detail": pong})

        ser.write(b"CMD:TIMEBASE_SELF_TEST\r\n")
        timebase = wait_for(ser, "ACK:TIMEBASE_SELF_TEST", 3.0, log)
        timebase_match = re.search(
            r"running=(\d+) min_delta=(\d+) max_delta=(\d+) pass=(\d+)",
            timebase or "",
        )
        timebase_ok = bool(
            timebase_match
            and timebase_match.group(1) == "1"
            and int(timebase_match.group(2)) > 0
            and timebase_match.group(4) == "1"
        )
        checks.append({"name": "timebase", "ok": timebase_ok, "detail": timebase})

        ser.write(b"CMD:SPI_ANCHOR_SELF_TEST,1000\r\n")
        anchor = wait_for(ser, "ACK:SPI_ANCHOR_SELF_TEST", 5.0, log)
        anchor_match = re.search(
            r"count=(\d+) min_low_cycles=(\d+) max_low_cycles=(\d+) "
            r"max_uncertainty_cycles=(\d+) pass=(\d+) bench=BENCH_PENDING",
            anchor or "",
        )
        anchor_ok = bool(
            anchor_match
            and anchor_match.group(1) == "1000"
            and int(anchor_match.group(2)) >= 64
            and anchor_match.group(5) == "1"
        )
        checks.append({"name": "spi_anchor", "ok": anchor_ok, "detail": anchor})

        ser.write(b"CMD:FPGA_STATUS\r\n")
        fpga = wait_for(ser, "ACK:FPGA_STATUS", 3.0, log)
        fpga_ok = bool(fpga and "id=0x2023(0)" in fpga and "ctrl_en=1(0)" in fpga)
        checks.append({"name": "fpga_compat", "ok": fpga_ok, "detail": fpga})

        ser.write(b"CMD:SPI_LL_STATUS\r\n")
        ll_status = wait_for(ser, "ACK:SPI_LL_STATUS", 3.0, log)
        ll_match = re.search(
            r"transport=LL_POLL transfers=(\d+) timeouts=(\d+) errors=(\d+) pass=(\d+)",
            ll_status or "",
        )
        ll_ok = bool(
            ll_match
            and int(ll_match.group(1)) > 0
            and ll_match.group(2) == "0"
            and ll_match.group(3) == "0"
            and ll_match.group(4) == "1"
        )
        checks.append({"name": "spi_ll_transport", "ok": ll_ok, "detail": ll_status})

    passed = all(check["ok"] for check in checks)
    print(json.dumps({"status": "PASS" if passed else "FAIL", "checks": checks,
                      "log_tail": log[-30:]}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    sys.exit(run(args.port, args.baud))
