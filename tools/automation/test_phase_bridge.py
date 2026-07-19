import csv
import json
import math
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "host_phase_bridge"
VECTORS = BUILD / "phase_bridge_vectors.csv"
EXE = BUILD / "test_phase_bridge.exe"
TWO_PI = 2.0 * math.pi


def wrap_pi(value):
    wrapped = math.remainder(value, TWO_PI)
    if wrapped <= -math.pi:
        wrapped += TWO_PI
    if wrapped > math.pi:
        wrapped -= TWO_PI
    return wrapped


def phase_to_u32(phase):
    return int((phase % TWO_PI) * (2**32) / TWO_PI) & 0xFFFFFFFF


def make_case(name, frequency, sample_rate, cosine_phase0, adc_t0, anchor,
              fpga_phase, calibration=0.0, uncertainty=100, limit=256):
    raw_goertzel = wrap_pi(cosine_phase0 - TWO_PI * frequency / sample_rate)
    elapsed = (anchor - adc_t0) & 0xFFFFFFFF
    valid = uncertainty <= limit and elapsed <= 0x7FFFFFFF and frequency > 0
    if valid:
        adc_anchor = cosine_phase0 + TWO_PI * frequency * elapsed / 480_000_000.0 + calibration
        expected_error = wrap_pi(adc_anchor - fpga_phase)
    else:
        expected_error = 0.0
    return [name, raw_goertzel, frequency, sample_rate, adc_t0, anchor, 480_000_000,
            phase_to_u32(fpga_phase), calibration, uncertainty, limit,
            int(valid), expected_error]


def generate_vectors():
    cases = [
        make_case("20k_basic", 20_000.0, 2_500_000.0, 0.3, 1000, 25000, -0.4),
        make_case("50k_near_pos_pi", 50_000.0, 2_500_000.0, math.pi - 1e-5,
                  2000, 42000, -math.pi + 2e-5),
        make_case("100k_near_neg_pi", 100_000.0, 2_500_000.0, -math.pi + 2e-5,
                  3000, 51000, math.pi - 1e-5),
        make_case("counter_wrap", 50_000.0, 2_500_000.0, -1.1,
                  0xFFFFFF00, 0x00002000, 1.4),
        make_case("positive_offset", 50_010.0, 2_500_000.0, 0.8, 9000, 900000, 0.1),
        make_case("negative_offset", 49_990.0, 2_500_000.0, -0.7, 9000, 900000, -0.2),
        make_case("calibration", 50_000.0, 2_500_000.0, 0.2, 10, 100010,
                  -0.3, calibration=0.125),
        make_case("uncertainty_reject", 50_000.0, 2_500_000.0, 0.0, 10, 1000,
                  0.0, uncertainty=300, limit=256),
        make_case("ambiguous_time_reject", 50_000.0, 2_500_000.0, 0.0,
                  0x100, 0x80000101, 0.0),
    ]
    BUILD.mkdir(parents=True, exist_ok=True)
    with VECTORS.open("w", newline="", encoding="ascii") as stream:
        writer = csv.writer(stream)
        writer.writerow(["name", "raw_phase", "frequency", "sample_rate", "adc_t0",
                         "anchor", "core_clock", "fpga_phase", "calibration",
                         "uncertainty", "max_uncertainty", "expected_valid",
                         "expected_error"])
        writer.writerows(cases)
    return len(cases)


def main():
    case_count = generate_vectors()
    gcc = shutil.which("gcc")
    if not gcc:
        print(json.dumps({"status": "FAIL", "error": "host gcc not found"}, indent=2))
        return 2
    compile_cmd = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
                   "-I", str(ROOT / "Core" / "Algorithms"),
                   str(ROOT / "Core" / "Algorithms" / "phase_bridge.c"),
                   str(ROOT / "tests" / "host" / "test_phase_bridge.c"),
                   "-lm", "-o", str(EXE)]
    compiled = subprocess.run(compile_cmd, cwd=ROOT, text=True, capture_output=True)
    if compiled.returncode != 0:
        print(json.dumps({"status": "FAIL", "stage": "compile",
                          "stdout": compiled.stdout, "stderr": compiled.stderr}, indent=2))
        return 1
    tested = subprocess.run([str(EXE), str(VECTORS)], cwd=ROOT, text=True, capture_output=True)
    passed = tested.returncode == 0 and "DPLL_DATASET_PASS" in tested.stdout
    print(json.dumps({"status": "PASS" if passed else "FAIL", "cases": case_count,
                      "compile": "PASS", "test_output": tested.stdout.strip(),
                      "stderr": tested.stderr.strip()}, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
