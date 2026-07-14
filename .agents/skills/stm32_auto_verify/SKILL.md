---
name: stm32_auto_verify
description: A closed-loop development skill allowing the AI to compile, flash, and verify STM32 firmware using UART loopback automatically.
---
# STM32 Auto Verify Skill

## Overview
This skill empowers the AI to independently verify code modifications on STM32 hardware without user intervention. By utilizing the `tools/automation/` Python scripts, the AI can perform a complete validation loop: Code -> Build -> Flash -> Serial Test -> Results.

## Prerequisites
1. Python environment with `pyserial` installed (see `tools/automation/requirements.txt`).
2. STM32 ST-Link connected and powered.
3. UART connected to the host on the specified COM port (default `COM16`, `115200`).

## The Automation Loop
When modifying STM32 features, you should automatically test them using this sequence:

1. **Build and Flash**:
   Run the following command to compile using CMake/Ninja and flash via OpenOCD:
   ```powershell
   python tools/automation/build_and_flash.py
   ```
   *Note: Check the output to ensure `Flash successful.`*

2. **Serial Functional Testing**:
   Run the test runner to verify the firmware logic:
   ```powershell
   python tools/automation/test_runner.py --port COM16 --baud 115200
   ```
   *Note: The script outputs JSON indicating `PASS` or `FAIL` and logs any structured data.*

## Firmware Protocol Rules
To utilize the auto-verify scripts, the STM32 firmware MUST implement the following ASCII line-based protocol over UART:
- **Host -> STM32**: `CMD:<CommandName> [Params]\n` (e.g., `CMD:PING`)
- **STM32 -> Host**: `ACK:<CommandName> [Result]\n` (e.g., `ACK:PONG`)
- **STM32 -> Host**: `LOG:[INFO|WARN|ERR] <Message>\n`

When extending the firmware, add parsers for your new `CMD:` and verify them by expanding `test_runner.py` or writing a custom Python test script.
