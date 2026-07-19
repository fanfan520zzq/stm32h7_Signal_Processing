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

typedef struct {
    uint16_t id;
    uint16_t protocol_version;
    uint16_t capabilities;
    uint16_t status;
    uint16_t protocol_error_count;
    uint16_t build_id;
} FPGA_Info_t;

typedef struct {
    uint16_t sequence;
    uint16_t status;
    uint16_t protocol_error_count;
    uint64_t sample_counter;
    uint64_t apply_counter;
    uint32_t phase_a;
    uint32_t phase_b;
    uint32_t active_ftw_a;
    uint32_t active_ftw_b;
} FPGA_Snapshot_t;

typedef struct {
    uint32_t ftw_a;
    uint32_t ftw_b;
    uint32_t phase_offset_b;
    uint8_t ratio_n;
    uint8_t derived_b_enable;
    uint8_t raw_ftw_enable;
} FPGA_DDSConfig_t;

typedef struct {
    uint16_t config_sequence;
    uint64_t apply_counter;
    uint32_t active_ftw_a;
    uint32_t active_ftw_b;
} FPGA_CommitReceipt_t;

int32_t FPGA_Ctrl_GetInfo(FPGA_Info_t *info);
int32_t FPGA_Ctrl_AcquireSnapshot(FPGA_Snapshot_t *snapshot);
int32_t FPGA_Ctrl_CommitDDSConfig(const FPGA_DDSConfig_t *config,
                                  FPGA_CommitReceipt_t *receipt);
void FPGA_Ctrl_PrintInfo(void);
void FPGA_Ctrl_PrintSnapshot(void);
void FPGA_Ctrl_RunProtocolSelfTest(uint32_t count);

#endif // FPGA_CTRL_H
