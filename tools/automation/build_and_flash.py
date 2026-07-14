import subprocess
import sys
import os

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
BUILD_DIR = os.path.join(PROJECT_ROOT, "build", "Debug")
ELF_PATH = os.path.join(BUILD_DIR, "IIT6_Oscilliscope.elf").replace('\\', '/')

def run_cmd(cmd, cwd=PROJECT_ROOT):
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
    if result.returncode != 0:
        print(f"Error executing command: {' '.join(cmd)}")
        print(result.stdout)
        print(result.stderr)
        sys.exit(1)
    else:
        # Optionally print stdout if we want to see CMake output
        print(result.stdout)
    return result

def build():
    print("--- Building Project ---")
    run_cmd(["cmake", "--preset", "Debug"])
    run_cmd(["cmake", "--build", "--preset", "Debug"])
    print("Build successful.\n")

def flash():
    print("--- Flashing Project ---")
    openocd_path = r"D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe"
    openocd_scripts = r"D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts"
    openocd_cfg = r"D:\openOCD\DevEnv\stm32h7_stlink.cfg"
    
    cmd = [
        openocd_path,
        "-s", openocd_scripts,
        "-f", openocd_cfg,
        "-c", "tcl_port disabled",
        "-c", "gdb_port disabled",
        "-c", f"program {ELF_PATH} reset exit"
    ]
    # program with verify sometimes requires specific formatting or syntax in openocd, just reset exit for now
    # OpenOCD's output is often on stderr
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=PROJECT_ROOT, text=True, capture_output=True)
    if result.returncode != 0:
        print("Error executing OpenOCD:")
        print(result.stdout)
        print(result.stderr)
        sys.exit(1)
    else:
        print(result.stderr) # openocd prints to stderr even on success
        print("Flash successful.\n")

if __name__ == "__main__":
    build()
    flash()
    print("=== ALL STAGES PASSED ===")
