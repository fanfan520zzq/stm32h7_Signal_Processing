import json
import shutil
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD = ROOT / "build" / "host_dpll_controller"
EXE = BUILD / "test_dpll_controller.exe"


def main():
    BUILD.mkdir(parents=True, exist_ok=True)
    gcc = shutil.which("gcc")
    if not gcc:
        print(json.dumps({"status": "FAIL", "error": "host gcc not found"}, indent=2))
        return 2
    command = [gcc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2",
               "-D_USE_MATH_DEFINES", "-I", str(ROOT / "Core" / "Algorithms"),
               str(ROOT / "Core" / "Algorithms" / "dpll_controller.c"),
               str(ROOT / "tests" / "host" / "test_dpll_controller.c"),
               "-lm", "-o", str(EXE)]
    compiled = subprocess.run(command, cwd=ROOT, text=True, capture_output=True)
    if compiled.returncode != 0:
        print(json.dumps({"status": "FAIL", "stage": "compile",
                          "stdout": compiled.stdout, "stderr": compiled.stderr}, indent=2))
        return 1
    tested = subprocess.run([str(EXE)], cwd=ROOT, text=True, capture_output=True)
    passed = tested.returncode == 0 and "DPLL_CONTROLLER_PASS" in tested.stdout
    print(json.dumps({"status": "PASS" if passed else "FAIL", "compile": "PASS",
                      "test_output": tested.stdout.strip(), "stderr": tested.stderr.strip()},
                     indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
