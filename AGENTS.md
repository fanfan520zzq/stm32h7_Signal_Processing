# AGENTS.md
- **Purpose**: Equip coding agents to extend the IIT6 STM32 oscilloscope/DDS firmware without re-discovering tribal knowledge.
- **Existing docs**: Only `README.md` exists; treat its UART frame tables and timer notes as canonical input specs.

## System Topology
- `Core/Src/main.c` boots HAL, configures clocks, then defers to FreeRTOS via `MX_FREERTOS_Init`; almost all behavior lives in `Core/Task/*.c`.
- Timing is split as in `README.md`: TIM3/TIM4 clock simultaneous ADC1/ADC2 sampling, TIM6/TIM7 feed the dual DDS channels, TIM8 remains the RTOS tick, and TIM1/5/12/13 are reserved for frequency metering/gate signals.
- `STM32H743XX_FLASH.ld` adds a dedicated `.dma_buffer` region in AXI SRAM; any large DMA buffer (ADC, DDS, FFT scratch) must use `__attribute__((section(".dma_buffer"), aligned(32)))` to avoid cache incoherency.
- `cmake/gcc-arm-none-eabi.cmake` + `CMakePresets.json` assume the Arm GNU toolchain and Ninja; keep user code in separate `Core/Task` units and list them in the root `CMakeLists.txt`.

## Task & Queue Model
- Queues live in `Core/Src/freertos.c`: `UARTQueue` streams raw bytes from `UART1_Receive_Start` (`Core/Src/usart.c`), `MSGQueue` shuttles decoded frames.
- `Core/Task/UARTTX.c` parses the `AA <len> <op> ...` frames described in `README.md`, checksums them, then heap-allocates an `APP_Text` (`MSG.h`) pushed to `MSGQueue`.
- `Core/Task/CMD.c` owns `DDS_Init`, switches `g_is_adc_continuous`, updates DAC waveforms, and controls when `ADCTask` is armed by releasing `ADCSEM`.
- Semaphores defined in `freertos.c`: `ADCSEM` starts a capture, `FFTSEM` signals that both ADC DMA buffers finished (`ADCTask.c`), and `ADCFinishedSem` is available for future gating but currently unused.

## Signal Chain
- `Core/Task/ADCTask.c` starts dual DMA conversions into `CH1_Buffer`/`CH2_Buffer` (`LEN=4096` from `ADCTask.h`) after synchronously starting TIM3/TIM4; the conversion callbacks stop DMA and release `FFTSEM` only when both channels complete.
- `Core/Task/FFTTask.c` is the heart of analysis: it invalidates stats, performs software zero-cross frequency measurement (with hysteresis), applies a cached Hanning window, runs `arm_rfft_fast_f32`, classifies wave type via harmonic ratios, then drives the LCD. Respect the constant sample rate `RATE=2000000.0f` when adjusting algorithms.
- Continuous streaming happens only when `g_is_adc_continuous==1`: `FFTTask` re-releases `ADCSEM`, so be careful to update that flag when adding stop/step features.

## Display & Host IO
- `Core/Task/LCD.c` assumes a Nextion-style UART screen: all strings end with `0xFF 0xFF 0xFF`, and waveform points are emitted via `add 1,<series>,<value>`; reuse `LCD_Output` rather than sprinkling raw `printf` commands.
- `DDS.c` generates two independent waveforms by precalculating 1024-sample LUTs and filling DMA buffers sized as `1e6/freq`; any new waveform type must write to both buffer arrays and flush DCache with `SCB_CleanDCache_by_Addr` before restarting DMA.

## Build & Flash Workflow
- Configure once with `cmake --preset Debug` (or other preset) to `build/<Preset>`; the toolchain file wires in STM32CubeMX sources plus CMSIS-DSP.
- Build with `cmake --build build/Debug` and flash/debug using your usual OpenOCD/STM32CubeProgrammer flow; the compiled ELF lives in `cmake-build-debug/IIT6_Oscilliscope.elf` when using CLion presets.
- The root `CMakeLists.txt` already links `Middlewares/ST/ARM/DSP/Lib/libarm_cortexM7lfdp_math.a` and enables `_printf_float`; keep that consistent if you add new targets.

## Coding Conventions & Pitfalls
- Keep FreeRTOS-facing functions as `Start<Name>Task` with infinite loops; reference them from `freertos.c` rather than creating tasks ad hoc.
- When touching DMA buffers, stop the peripheral (`HAL_ADC_Stop_DMA`, `HAL_DAC_Stop_DMA`) before editing and restart after cache maintenance, mirroring the patterns in `ADCTask.c` and `DDS.c`.
- Use heap allocations for incoming UART commands (see `pvPortMalloc` in `UARTTX.c`) and remember to free or recycle if you add long-lived control messages.
- Any new public message opcode belongs in `MSG.h` so the UART parser and CMD task stay in sync; keep opcodes aligned with the README frame table.

## AI Development Constraints
- Read `AI_DEVELOPMENT_RULES.md` before making implementation changes.
- Prefer local ST official materials under `C:\Users\Lenovo\STM32Cube\Repository`, especially H7 `Templates_LL` and `Examples_LL`, together with the manuals in `ref_doc`.
- When an official PDF, example, or driver source materially informs a module, add a module Markdown note with the exact source path, chapter/section/page or example path, symbol/register, applicability, and verification status.
- Treat direct official references as evidence level A; current-board verification as B; reviewed-but-unverified material as C; AI-only material as D.
- After each accepted refactoring stage, build, run the agreed hardware checks, update the verification record, create a Git commit/tag, and prepare the stable result for import into `D:\ZYNQ32_MEMRAX`.
- Do not guess OpenOCD, debugger, serial-port, board-reset, or ELF-path settings. Use the project's command manuals when they are added.

