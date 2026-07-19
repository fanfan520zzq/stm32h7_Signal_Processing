# Serial-triggered automatic run gate

## Scope

This batch replaces the planned physical start button with a UART command while preserving the
one-shot competition workflow: change the source parameters first, then issue one start command.

```text
CMD:AUTO_RUN_START,<phase_deg>
CMD:AUTO_RUN_STATUS
CMD:AUTO_RUN_STOP
```

The top-level `auto_run_service` performs one analysis, applies both reconstructed FPGA channels
with one atomic commit, configures the appropriate B mode, starts the A-channel DPLL and enforces
the 20-second initial-lock timeout.

This is not continuous hot-change detection. A source frequency or waveform change after lock
requires another `CMD:AUTO_RUN_START,...` command.

## UART traffic policy

- no periodic heartbeat, FFT or separation print in the normal idle profile;
- no received-command echo;
- immediate ACK, error and state-transition messages;
- one steady DPLL summary every five seconds;
- detailed status remains available on request.

## Automated gates

```powershell
python -m py_compile tools/automation/test_auto_run_protocol.py
python -m py_compile tools/automation/test_auto_run_board.py
cmake --build build/Debug
python tools/automation/build_and_flash.py
python tools/automation/test_auto_run_protocol.py --port COM16 --quiet-seconds 5
python tools/automation/test_auto_run_board.py --port COM16 --phase 0 --timeout 22
```

Results:

- firmware build: PASS;
- OpenOCD program/reset: PASS;
- five-second idle UART window: zero unsolicited lines;
- PING returned one ACK only;
- invalid 3-degree phase rejected;
- status and stop commands: PASS;
- current input automatically identified as A=30 kHz and B=50 kHz;
- B mode automatically selected as `COMMON_PPM`;
- automatic board run reached `LOCKED` in under two seconds.

Version-management rerun note: the controller test, B-mode test and firmware build passed again.
The final OpenOCD/COM16 repetition could not run because the connected ST-Link and COM16 were no
longer available (`open failed` / serial port not found). The board results above are from the
immediately preceding run of the same source and firmware in this development session, not from
that disconnected rerun.

## Evidence boundary

The automated board run proves the UART trigger, one-shot signal analysis, FPGA parameter apply,
DPLL start and digital lock-state path on the connected board. It does not characterize every
frequency/waveform combination, continuous source changes, analog amplitude, or connector-level
phase error. Those remain separate instrument tests.
