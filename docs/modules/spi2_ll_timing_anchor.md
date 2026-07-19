# SPI2 LL timing anchor

## Scope

Stage 07.3 first replaced only the timing-critical PF9 chip-select writes with LL GPIO BSRR access.
After that sub-step passed independently, the fixed four-byte SPI2 transfer was migrated from HAL
blocking transfer to LL TXP/RXP/EOT polling with a DWT cycle timeout. A DWT CYCCNT timebase also
timestamps a CS-low anchor pulse that contains no SCK edges.

## Official local references

- STM32CubeH7 V1.12.1 `Projects/NUCLEO-H743ZI/Examples_LL/SPI/SPI_FullDuplex_ComIT`:
  `LL_SPI_StartMasterTransfer`, TXP/RXP/EOT handling, and 8-bit TX/RX access.
- `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_spi.h`:
  `LL_SPI_TransmitData8`, `LL_SPI_ReceiveData8`, `LL_SPI_IsActiveFlag_TXP`,
  `LL_SPI_IsActiveFlag_RXP`, `LL_SPI_IsActiveFlag_EOT`, and
  `LL_SPI_StartMasterTransfer`.
- `Drivers/STM32H7xx_HAL_Driver/Inc/stm32h7xx_ll_gpio.h`:
  `LL_GPIO_ResetOutputPin` and `LL_GPIO_SetOutputPin` write GPIO BSRR directly.
- CMSIS `core_cm7.h`: `CoreDebug_DEMCR_TRCENA_Msk`, `DWT_CTRL_CYCCNTENA_Msk`, and `DWT->CYCCNT`.

## Project-specific mapping

- SPI instance: SPI2, mode 0, 8-bit, software NSS.
- SCK/MISO/MOSI: PB10/PB14/PC1.
- FPGA CS/anchor: PF9 GPIO output, active low.
- Minimum programmed anchor-low interval: 64 core cycles.
- Interrupts are masked only across the short timestamped anchor pulse and restored to their
  previous PRIMASK state.

## Verification boundary

`CMD:TIMEBASE_SELF_TEST` checks that CYCCNT advances and reports consecutive-read deltas.
`CMD:SPI_ANCHOR_SELF_TEST,1000` checks 1000 firmware-generated PF9 pulses and reports timestamp
envelopes. `CMD:SPI_LL_STATUS` reports completed transfers, timeouts and peripheral errors so the
host can prove that FPGA compatibility reads used the LL path. These are MCU-side digital
measurements. Physical PF9/SCK timing still requires a logic analyzer and remains `BENCH_PENDING`.
