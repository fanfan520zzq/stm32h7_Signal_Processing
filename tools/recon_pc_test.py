"""PC-side learning/reconstruction test runner for USART1.

Usage:
  python tools/recon_pc_test.py --port COM7 --learn-seconds 30 --recon-seconds 30
  python tools/recon_pc_test.py --port COM7 --recon-only --recon-seconds 60

The script sends the same 6-byte commands previously sent by the LCD, saves
the complete device log, and writes a compact Markdown report for review.
"""

from __future__ import annotations

import argparse
import json
import re
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise SystemExit("需要 pyserial：python -m pip install pyserial") from exc


CMD_RESET = bytes.fromhex("AD FF FF FF FF DA")
CMD_LEARN = bytes.fromhex("AD 45 FE FE FE DA")
CMD_RECON = bytes.fromhex("AD 67 EE EE EE DA")

RE_ANALYSIS = re.compile(
    r"RECON f0=([0-9.+-]+) input_vpp=([0-9.+-]+) dc=([0-9.+-]+) "
    r"phase=([0-9.+-]+)deg harmonics=(\d+)"
)
RE_PLL = re.compile(
    r"RECON PLL f0=([0-9.+-]+) out=([0-9.+-]+) "
    r"(?:corr=([0-9.+-]+)Hz )?err=([0-9.+-]+)deg ftw=(\d+) relock=(\d+)"
)
RE_HARM = re.compile(
    r"HARM k=(\d+) f=([0-9.+-]+) x_vpp=([0-9.+-]+) x_phase=([0-9.+-]+)deg"
)


def read_available(ser: serial.Serial, seconds: float) -> str:
    end = time.monotonic() + seconds
    chunks: list[bytes] = []
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            chunks.append(ser.read(n))
        else:
            time.sleep(0.01)
    return b"".join(chunks).decode("utf-8", errors="replace")


def read_until(ser: serial.Serial, marker: str, timeout: float) -> tuple[str, bool]:
    end = time.monotonic() + timeout
    data = bytearray()
    found = False
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            data.extend(ser.read(n))
            text = data.decode("utf-8", errors="replace")
            if marker in text:
                found = True
                data.extend(ser.read(ser.in_waiting))
                break
        else:
            time.sleep(0.01)
    return data.decode("utf-8", errors="replace"), found


def send(ser: serial.Serial, command: bytes) -> None:
    ser.write(command)
    ser.flush()


def make_report(raw: str) -> dict:
    analyses = [
        {"f0_hz": float(a), "input_vpp": float(b), "dc": float(c),
         "phase_deg": float(d), "harmonics": int(e)}
        for a, b, c, d, e in RE_ANALYSIS.findall(raw)
    ]
    pll = [
        {"f0_hz": float(a), "out_hz": float(b),
         "corr_hz": float(c) if c else None, "error_deg": float(d),
         "ftw": int(e), "relock": int(f)}
        for a, b, c, d, e, f in RE_PLL.findall(raw)
    ]
    harmonics = [
        {"k": int(a), "f_hz": float(b), "vpp": float(c), "phase_deg": float(d)}
        for a, b, c, d in RE_HARM.findall(raw)
    ]
    return {
        "analysis_count": len(analyses),
        "pll_count": len(pll),
        "analysis": analyses,
        "pll": pll,
        "harmonics": harmonics,
        "rebuild_count": raw.count("RECON_AUTO_REBUILD_START"),
        "relock_count": raw.count("RECON PLL relock"),
        "ready_count": raw.count("RECON_READY"),
        "reject_count": raw.count("RECON reject"),
        "raw_bytes": len(raw.encode("utf-8")),
    }


def markdown(report: dict) -> str:
    lines = ["# Reconstruction PC test report", "", f"- analysis blocks: {report['analysis_count']}",
             f"- PLL blocks: {report['pll_count']}", f"- RECON_READY: {report['ready_count']}",
             f"- auto rebuilds: {report['rebuild_count']}", f"- relocks: {report['relock_count']}",
             f"- rejects: {report['reject_count']}", ""]
    if report["analysis"]:
        a = report["analysis"][-1]
        lines += ["## Last analysis", "", f"`f0={a['f0_hz']:.3f} Hz, input_vpp={a['input_vpp']:.4f} V, "
                  f"phase={a['phase_deg']:.3f} deg, harmonics={a['harmonics']}`", ""]
    if report["pll"]:
        lines += ["## PLL samples", "", "| f0 Hz | output Hz | error deg | relock |", "|---:|---:|---:|---:|"]
        for p in report["pll"][-20:]:
            lines.append(f"| {p['f0_hz']:.3f} | {p['out_hz']:.6f} | {p['error_deg']:.4f} | {p['relock']} |")
    lines += ["", "## Interpretation", "", "原始日志请结合示波器波形一起分析；不要只用次数判断锁定。"]
    return "\n".join(lines) + "\n"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--learn-seconds", type=float, default=40.0)
    ap.add_argument("--recon-seconds", type=float, default=30.0)
    ap.add_argument("--recon-only", action="store_true",
                    help="skip learning and directly send reconstruction command")
    ap.add_argument("--no-reset", action="store_true",
                    help="do not send reset before reconstruction")
    ap.add_argument("--out-dir", default="pc_test_reports")
    args = ap.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")

    if args.recon_only:
        with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
            ser.reset_input_buffer()
            if not args.no_reset:
                send(ser, CMD_RESET)
                time.sleep(0.3)
            print("跳过学习，直接启动已存储表的重建。")
            send(ser, CMD_RECON)
            recon_log = read_available(ser, args.recon_seconds)

        raw = "=== PC_RECON_ONLY ===\n" + recon_log
        report = make_report(raw)
        (out / f"recon_{stamp}.log").write_text(raw, encoding="utf-8")
        (out / f"recon_{stamp}.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
        (out / f"recon_{stamp}.md").write_text(markdown(report), encoding="utf-8")
        print(f"报告已保存到：{out.resolve()}")
        return

    with serial.Serial(args.port, args.baud, timeout=0.05) as ser:
        ser.reset_input_buffer()
        send(ser, CMD_RESET)
        time.sleep(0.3)
        send(ser, CMD_LEARN)
        learn_log, learn_done = read_until(ser, "=== SWEEP_DONE", args.learn_seconds)
        if not learn_done:
            print("警告：学习超时，未检测到 === SWEEP_DONE；仍继续等待换线。")
        else:
            time.sleep(1.0)
            learn_log += read_available(ser, 0.2)
        print("学习阶段采集完成，请现在改变物理接线。")
        input("接线完成后按 Enter 开始重建：")
        send(ser, CMD_RECON)
        recon_log = read_available(ser, args.recon_seconds)

    raw = learn_log + "\n=== PC_RECON_PHASE ===\n" + recon_log
    report = make_report(raw)
    (out / f"recon_{stamp}.log").write_text(raw, encoding="utf-8")
    (out / f"recon_{stamp}.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    (out / f"recon_{stamp}.md").write_text(markdown(report), encoding="utf-8")
    print(f"报告已保存到：{out.resolve()}")


if __name__ == "__main__":
    main()
