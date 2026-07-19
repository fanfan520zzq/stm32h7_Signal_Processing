#ifndef FPGA_CTRL_H
#define FPGA_CTRL_H

#include <stdint.h>
#include "dft_separate.h"

// Initialize SPI driver, enable FPGA SPI override control, and verify
// the link by reading back ID (0x00) and CTRL_EN (0x02).
void FPGA_Ctrl_Init(void);

// Set wave parameters (not committed immediately). Returns 0 on success.
int32_t FPGA_Ctrl_SetFreq(uint8_t ch, uint32_t freq_hz);
int32_t FPGA_Ctrl_SetWave(uint8_t ch, uint8_t wave_type, uint8_t vpp);

// Commit all shadow registers to active registers
void FPGA_Ctrl_Commit(void);

// Synchronize phases (reset phase accumulators)
void FPGA_Ctrl_Sync(void);

// Top level function for algorithm integration (single channel helper)
void FPGA_Ctrl_UpdateFromAlgorithm(uint8_t ch, uint8_t wave_type, uint32_t freq_hz);

// Apply a full separation result: sig1 -> CH1, sig2 -> CH2, one COMMIT.
// Returns number of failed SPI transactions (0 = all OK).
int32_t FPGA_Ctrl_ApplyResult(const SignalSeparationResult *res);

// Print FPGA link status as ACK:FPGA_STATUS ... (used by UART command)
void FPGA_Ctrl_PrintStatus(void);

// Cumulative SPI transaction error count (HAL errors + RX CRC failures)
extern volatile uint32_t fpga_spi_err_count;

#endif // FPGA_CTRL_H
