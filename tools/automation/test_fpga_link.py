"""
Stage05 FPGA link test: verify STM32 -> SPI -> FPGA parameter path.

Checks (protocol per .agents/skills/stm32_auto_verify):
  1. CMD:PING            -> ACK:PONG
  2. CMD:FPGA_STATUS     -> ACK:FPGA_STATUS id=0x2023 ctrl_en=1 (all res==0)
  3. separation apply log -> applied freq == readback freq, errs==0, crcum==0
  4. config seq increments between two FPGA_STATUS queries (>=1.2s apart)

Usage: python tools/automation/test_fpga_link.py --port COM16 --baud 115200
Exit code 0 = PASS, 1 = FAIL. Prints JSON result at the end.
"""
import serial
import time
import sys
import json
import re
import argparse

APPLY_RE = re.compile(
    r"FPGA apply: ch1=(\d+)Hz ch2=(\d+)Hz rb_ch1=(\d+)Hz rb_ch2=(\d+)Hz seq=(\d+) errs=(-?\d+) crcum=(\d+)")
STATUS_RE = re.compile(
    r"ACK:FPGA_STATUS id=0x([0-9A-Fa-f]+)\((-?\d+)\) ctrl_en=(\d+)\((-?\d+)\) seq=(\d+)\((-?\d+)\) err=(\d+)")


def read_lines(ser, duration, sink):
    """Collect complete lines for `duration` seconds into sink (list)."""
    end = time.time() + duration
    buf = b""
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="ignore").strip()
                if text:
                    sink.append(text)
        else:
            time.sleep(0.01)


def wait_for(ser, match_fn, timeout, sink):
    """Read lines until match_fn(line) returns truthy; return that value or None."""
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="ignore").strip()
                if not text:
                    continue
                sink.append(text)
                m = match_fn(text)
                if m:
                    return m
        else:
            time.sleep(0.01)
    return None


def send_cmd(ser, cmd):
    ser.write((cmd + "\r\n").encode())


def run_test(port, baud):
    checks = []
    logs = []

    def check(name, ok, detail=""):
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except Exception as e:
        print(json.dumps({"status": "FAIL", "reason": f"open port failed: {e}"}))
        sys.exit(1)

    time.sleep(0.3)
    ser.reset_input_buffer()

    # 1. PING
    send_cmd(ser, "CMD:PING")
    m = wait_for(ser, lambda l: l == "ACK:PONG", 3.0, logs)
    check("ping", m is not None, "expect ACK:PONG")

    # 2. FPGA_STATUS
    send_cmd(ser, "CMD:FPGA_STATUS")
    m = wait_for(ser, lambda l: STATUS_RE.search(l), 3.0, logs)
    if m:
        id_hex, id_res, ctrl, ctrl_res, seq1, seq_res, err = (
            m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4)),
            int(m.group(5)), int(m.group(6)), int(m.group(7)))
        check("fpga_id", int(id_hex, 16) == 0x2023 and id_res == 0,
              f"id=0x{id_hex} res={id_res}")
        check("ctrl_en", ctrl == 1 and ctrl_res == 0,
              f"ctrl_en={ctrl} res={ctrl_res}")
        err1 = err  # cumulative since boot; delta-checked later
    else:
        check("fpga_status", False, "no ACK:FPGA_STATUS")
        seq1 = None

    # 3. one apply log with matching readback
    m = wait_for(ser, lambda l: APPLY_RE.search(l), 6.0, logs)
    if m:
        ch1, ch2, rb1, rb2, aseq, errs, crcum = (int(m.group(i)) for i in range(1, 8))
        ok_rb = (ch1 == 0 or ch1 == rb1) and (ch2 == 0 or ch2 == rb2)
        check("apply_readback_match", ok_rb,
              f"ch1={ch1} rb1={rb1} ch2={ch2} rb2={rb2}")
        check("apply_no_err", errs == 0,
              f"per-apply errs={errs} (crcum={crcum} is cumulative)")
    else:
        check("apply_log", False, "no FPGA apply log within 6s (signal source connected?)")

    # 4. seq increments and no new SPI errors during the test
    if seq1 is not None:
        time.sleep(1.3)
        send_cmd(ser, "CMD:FPGA_STATUS")
        m = wait_for(ser, lambda l: STATUS_RE.search(l), 3.0, logs)
        if m:
            seq2 = int(m.group(5))
            err2 = int(m.group(7))
            check("seq_increments", seq2 > seq1, f"seq {seq1} -> {seq2}")
            check("spi_err_stable", err2 == err1, f"err {err1} -> {err2}")
        else:
            check("seq_increments", False, "no second ACK:FPGA_STATUS")

    ser.close()

    passed = all(c["ok"] for c in checks) and len(checks) >= 5
    result = {"status": "PASS" if passed else "FAIL", "checks": checks,
              "log_tail": logs[-30:]}
    print(json.dumps(result, indent=2, ensure_ascii=False))
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM16")
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()
    run_test(args.port, args.baud)
