# Stage 07.3 LL timing anchor verification

## Implemented scope

- Added a DWT CYCCNT `timebase_driver`.
- Fixed `dwt_start_time`: DMA is armed first, TIM4 is stopped/cleared, the frame timestamp is
  recorded, and TIM4 is then started.
- Replaced PF9 chip-select writes in the existing HAL SPI transaction with LL GPIO BSRR access.
- Added a timestamped, no-SCK CS anchor pulse with short interrupt masking.
- Added `CMD:TIMEBASE_SELF_TEST` and `CMD:SPI_ANCHOR_SELF_TEST,<count>`.
- Added an autonomous host test at `tools/automation/test_stage07_3_timing.py`.

This sub-step was verified before the SPI payload transfer changed. The fixed four-byte SPI2
transfer was then migrated to LL TXP/RXP/EOT polling with a DWT-cycle timeout and verified again.

## Automated board evidence

Build and flash command:

```powershell
python tools/automation/build_and_flash.py
```

Result: CMake/Ninja build passed, OpenOCD programming completed, and target reset succeeded.

UART command:

```powershell
python tools/automation/test_stage07_3_timing.py --port COM16 --baud 115200
```

Final result: JSON `PASS`, exit code 0.

- PING: PASS.
- DWT running: 1; final consecutive-read delta min/max: 108/174 cycles; PASS.
- 1000 CS anchors: final min/max low interval 103/141 cycles.
- Maximum timestamp edge-envelope uncertainty: 229 cycles; PASS against 256-cycle gate.
- FPGA compatibility status: ID `0x2023`, `ctrl_en=1`, SPI error count 0.
- LL transport status after compatibility access: 123 completed transfers, 0 timeouts, 0
  peripheral errors.

The first board run correctly failed the 256-cycle uncertainty gate at 298 cycles because the
Debug build called a non-inlined timestamp accessor inside the critical section. Direct CYCCNT
reads reduced the final maximum to 178 cycles without loosening the gate.

The pre-existing `test_fpga_link.py` regression also passed after LL migration: 30/50 kHz
write/readback matched, sequence advanced 640 to 642, and the cumulative SPI error count remained
zero. ADC frame timing is now nonzero and observed at 389168/389182 core cycles in those two
captures, replacing the previous broken zero-cycle observation.

## Evidence boundary

This proves firmware-observed timing and the existing STM32-SPI-FPGA compatibility link on the
current board. It does not measure the physical PF9 waveform; logic-analyzer validation remains
`BENCH_PENDING`. It also does not yet prove the v2 snapshot protocol or closed-loop DPLL.
