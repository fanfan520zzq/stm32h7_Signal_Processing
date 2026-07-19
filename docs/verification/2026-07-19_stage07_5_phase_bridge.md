# Stage 07.5 Phase Bridge algorithm verification

## Scope

- Added ADC capture metadata: DWT `adc_t0_cycles`, actual sample rate and frame sequence.
- Defined the Goertzel phase reference explicitly. The existing recurrence output is offset by
  `-omega`; `Goertzel_PhaseAtSample0Cosine` compensates it for a cosine phase referenced to ADC
  sample zero.
- Added pure `phase_bridge` C code for time extrapolation, DDS phase conversion, calibration,
  `(-pi, pi]` wrapping, DWT counter wrap and timestamp-uncertainty rejection.
- Added local anchor timestamp and uncertainty to the typed FPGA snapshot.
- No PI controller is connected and this stage never writes a tracking FTW.

Error sign convention: `wrap(input_phase - fpga_output_phase)`. A positive error means the input
leads the FPGA output.

## Offline golden-vector evidence

```powershell
python tools/automation/test_phase_bridge.py
```

The script generates vectors from Python double-precision equations, compiles the production C
algorithm with host GCC using `-Wall -Wextra -Werror`, and compares the C output with the golden
results.

Result: `DPLL_DATASET_PASS cases=9 max_error=2.94446945e-05`, exit code 0.

Coverage includes 20/50/100 kHz, phases adjacent to `-pi/pi`, DWT counter wrap, positive and
negative frequency offsets, calibration phase, uncertainty rejection and ambiguous-time rejection.
The acceptance limit is `2e-4 rad`, which is a numerical implementation limit only.

## Embedded build and evidence boundary

The STM32 cross-build includes the same `phase_bridge.c` and passes. The image was programmed with
OpenOCD, and the 1000-iteration Stage 07.4 board regression passed after the typed-snapshot change.
The snapshot reported a local anchor timestamp and 100-cycle falling-edge uncertainty while all
snapshot/protocol checks remained at zero errors.

This stage does not claim analog phase accuracy, PI convergence or `BENCH_PASS`.
