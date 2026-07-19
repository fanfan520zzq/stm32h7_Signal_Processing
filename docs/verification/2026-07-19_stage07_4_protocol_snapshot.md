# Stage 07.4 protocol and snapshot verification

## Engineering structure

- `Core/Drivers/spi_driver`: PF9 LL chip select, no-SCK anchor, fixed-frame LL SPI2 polling,
  DWT cycle timeout and transport counters.
- `Core/Protocols/fpga_spi_protocol`: CRC-8/ATM plus 16/32/64-bit little-word-order register
  access.
- `Core/Features/fpga_link/fpga_ctrl`: typed FPGA info, snapshot, DDS configuration and atomic
  commit receipt APIs.
- `tools/automation/test_stage07_4_protocol.py`: UART-driven board verification.

No CRC or register address remains in the SPI driver. The old `PROFILE_SPI_TEST` bypass was also
migrated to the protocol API.

## Build and flash

```powershell
python tools/automation/build_and_flash.py
```

Result: build passed, OpenOCD programming passed, target reset passed. Final image used 183668
bytes of flash (8.76%).

## Board protocol test

```powershell
python tools/automation/test_stage07_4_protocol.py --port COM16 --baud 115200 --count 1000
```

Result: JSON `PASS`, exit code 0.

- FPGA ID `0x2023`, protocol `0x0002`, capabilities `0x001F`, build `0x0721`.
- Atomic snapshot returned valid 64-bit sample/apply counters, 32-bit phases and active FTWs.
- 1000 iterations: 0 snapshot, sequence, counter, phase-identity and status-read errors.
- Raw FTW 32-bit write/readback, atomic COMMIT receipt and compatibility-mode restoration: 0
  errors.
- LL transport after the test: 25314 completed frames, 0 timeouts, 0 peripheral errors.

The first run exposed only a UART formatting defect: newlib-nano did not support `%llu` and
shifted later variadic arguments. The command was changed to print each 64-bit counter as two
32-bit hexadecimal halves; firmware protocol checks themselves had already reported zero errors.

## Regressions

- `test_stage07_3_timing.py`: PASS; 1000 anchors, maximum timestamp envelope 218 cycles,
  `BENCH_PENDING` for physical edges.
- `test_fpga_link.py`: PASS; 30/50 kHz compatibility write/readback, sequence 1400 to 1402,
  cumulative SPI errors remained zero.
- ADC DWT frame measurement remained active at approximately 389204 to 389210 core cycles.

## Evidence boundary

This proves the current board's digital STM32-SPI-FPGA v2 snapshot and atomic-control path. It
does not prove physical PF9/SCK timing, analog output phase, open-loop drift, or DPLL lock. Those
remain later-stage bench and algorithm gates.
