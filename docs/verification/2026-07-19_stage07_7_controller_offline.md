# Stage 07.7 DPLL controller and board closed-loop gate

## Scope

This batch adds a unit-aware discrete PI controller and connects it to an explicit, opt-in board
service command.
The input is wrapped phase error in radians and the output is a raw DDS FTW. Positive
`input-output` error increases FTW.

The controller implements:

- ACQUIRE, TRACK, LOCKED, HOLDOVER and LOST states;
- proportional gain in FTW/rad and integral gain in FTW/(rad*s);
- nominal-relative +/-200 ppm correction clamp;
- per-update FTW slew limit;
- conditional-integration anti-windup;
- frozen integrator/output during invalid measurements;
- one phase-load request per acquisition, never periodic phase jumps.

Initial tuning uses a 100 Hz update, an approximately 1 Hz natural-frequency starting point and
damping near 0.7. These values passed the digital loopback gate but remain subject to instrumented
board characterization.

## Host simulation

```powershell
python tools/automation/test_dpll_controller.py
```

The script compiles the production controller and plant test with host GCC under
`-Wall -Wextra -Werror`.

Result:

```text
DPLL_CONTROLLER_PASS lock_pos=1.00s lock_neg=1.17s
rms_pos_deg=0.0121 rms_neg_deg=0.0121
holdover=1 lost=1 recovered=1 saturation=4928
```

Checks include:

- positive error causes positive FTW correction;
- +100 and -100 ppm acquire LOCKED in less than 20 seconds;
- final numerical RMS and peak error remain within the simulation thresholds;
- 300 ppm plant mismatch cannot escape the +/-200 ppm FTW boundary;
- forced same-sign error exercises saturation and bounded anti-windup;
- NaN/invalid measurement freezes FTW and integrator in HOLDOVER;
- prolonged loss enters LOST and valid input recovers through a legal path;
- phase-load request occurs once per acquisition only.

## Board closed-loop gate

Commands:

```text
CMD:DPLL_CONFIG,30000,50000,100
CMD:DPLL_CLOSED_LOOP_START
CMD:DPLL_FAULT_INJECT,60
CMD:DPLL_STATUS
CMD:DPLL_OPEN_LOOP_STOP
```

Automated test:

```powershell
python tools/automation/test_stage07_7_closed_loop.py --port COM16
```

Final result: `PASS`.

- The controller acquired `LOCKED` from the configured 30 kHz nominal FTW
  (`0x00275254`).
- 29 locked log samples measured `0.1239 deg` RMS and `0.2551 deg` peak numerical
  phase error.
- A 60-update injected outage exercised
  `LOCKED -> HOLDOVER -> LOST -> ACQUIRE -> TRACK -> LOCKED`.
- FTW remained inside the nominal-relative +/-200 ppm bound.
- Snapshot anchor uncertainty was 110 core cycles, below the 256-cycle gate.
- Snapshot, phase, sequence-read and commit failure counters all remained zero outside the
  deliberate fault injection.
- The FPGA cumulative protocol CRC counter remained unchanged at `110 -> 110` during the final
  run. The nonzero baseline was accumulated during failed diagnostic runs before the timing fix.
- The corrected LL transaction order is: configure and preload TX FIFO while CS is high, assert
  CS, issue CSTART and transfer 32 bits, deassert CS, then disable SPI. This removed the prior
  first-frame CRC failures under the closed-loop workload.

The controller exposes a one-shot phase-load request during acquisition, but the board service
does not execute a periodic or acquisition-time `SYNC`; the demonstrated loop converges by FTW
control alone.

Regression after the timing change:

- Stage 07.4: 1000 atomic snapshots, raw FTW test and LL status `PASS`; 0 timeouts and 0 peripheral
  errors.
- Stage 07.6: open-loop status `PASS`, configuration sequence unchanged and `write_free=1`.
- Firmware build and OpenOCD flash both passed.

## Evidence boundary

Host simulation proves controller math and bounded state behavior. The UART test proves digital
closed-loop execution on the current STM32/FPGA board and the reported numerical phase error. It
does not prove analog DA phase accuracy, final gain selection across operating conditions, or the
5-degree instrument requirement. Those claims remain `BENCH_PENDING` until oscilloscope evidence
is recorded.

## User board observation

After explicitly enabling the closed loop, the user observed on the oscilloscope that reference
A and generated A-prime stopped drifting and were visibly locked. This upgrades the result from a
UART-only observation, but it is not yet a quantitative `BENCH_PASS` because no screenshot,
measurement conditions, lock time or phase-error value was recorded.

Changing reference A from triangle to sine did not change the generated A-prime waveform type.
That behavior is outside the Stage 07.7 DPLL contract: the loop tracks the configured fundamental
frequency and phase, while waveform classification and automatic waveform-type propagation are
not implemented. A continuing phase drift while `mode=2 state=LOCKED` would be a DPLL failure and
must be treated separately from waveform-type following.
