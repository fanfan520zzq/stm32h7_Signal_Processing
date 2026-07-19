# Stage 07.6 open-loop drift observation

## Scope

- Added `PROFILE_SPI_DPLL` and `dpll_service`.
- Added `CMD:DPLL_CONFIG`, `CMD:DPLL_OPEN_LOOP_START`, `CMD:DPLL_OPEN_LOOP_STOP` and
  `CMD:DPLL_STATUS`.
- Each accepted update measures the configured A frequency, acquires an atomic FPGA snapshot,
  runs the verified Phase Bridge, unwraps phase error and estimates frequency offset in ppm.
- Open-loop mode contains no FTW or COMMIT call. FPGA `CONFIG_SEQ` is checked continuously and
  reported as `write_free` evidence.

## Board evidence

```powershell
python tools/automation/test_stage07_6_open_loop.py --port COM16 --baud 115200
```

Result: JSON `PASS`, exit code 0.

Configuration: A=30 kHz, B=50 kHz, observation update=10 Hz.

- Six valid observations, zero rejected frames.
- Five captured consecutive log points showed wrapped error increasing from 0.3838664 to
  0.8717437 rad.
- Short-window frequency-offset estimates ranged approximately 5.741 to 6.566 ppm.
- Anchor falling-edge uncertainty was 96 core cycles for every captured point.
- Active FTW A remained `0x00275254`.
- FPGA `CONFIG_SEQ` remained exactly 2537 before, during and after observation.
- `write_free=1`; no COMMIT or FTW modification occurred.

## Evidence boundary

The ppm values are short-duration digital observations based on the current phase convention and
uncalibrated fixed latency. They are not oscillator metrology or analog-output accuracy claims.
No PI controller is connected, no lock is claimed, and physical waveform acceptance remains
`BENCH_PENDING`.
