import json
import pathlib
import subprocess
import sys
import tempfile


def run():
    root = pathlib.Path(__file__).resolve().parents[2]
    with tempfile.TemporaryDirectory(prefix="dpll_b_mode_") as tmp:
        exe = pathlib.Path(tmp) / "test_dpll_b_mode.exe"
        command = [
            "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror",
            "-I", str(root / "Core" / "Algorithms"),
            str(root / "Core" / "Algorithms" / "dpll_b_mode.c"),
            str(root / "tests" / "host" / "test_dpll_b_mode.c"),
            "-o", str(exe),
        ]
        compiled = subprocess.run(command, capture_output=True, text=True)
        if compiled.returncode != 0:
            print(json.dumps({"status": "FAIL", "stage": "compile",
                              "stderr": compiled.stderr}, indent=2))
            return 1
        tested = subprocess.run([str(exe)], capture_output=True, text=True)
        passed = tested.returncode == 0 and "DPLL_B_MODE_PASS" in tested.stdout
        print(json.dumps({"status": "PASS" if passed else "FAIL",
                          "output": tested.stdout.strip(), "stderr": tested.stderr}, indent=2))
        return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(run())
