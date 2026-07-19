#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "main.h"

typedef struct {
    uint32_t envelope_start;
    uint32_t cs_low_observed;
    uint32_t cs_high_requested;
    uint32_t envelope_end;
    uint32_t low_cycles;
    uint32_t uncertainty_cycles;
} SPI_AnchorTiming_t;

// Initialize SPI driver (if needed beyond CubeMX)
void SPI_Driver_Init(void);

// Basic blocking Transmit and Receive for SPI2
int32_t SPI_Driver_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
void SPI_Driver_GetLLStats(uint32_t *transfers, uint32_t *timeouts, uint32_t *errors);

// Generate a CS-low anchor without SCK and timestamp both GPIO edges.
int32_t SPI_Driver_AnchorPulse(uint32_t min_low_cycles, SPI_AnchorTiming_t *timing);

// Execute one complete FPGA frame including CS timing. CRC belongs to protocol layer.
int32_t SPI_Driver_TransferFrame(const uint8_t tx[4], uint8_t rx[4]);

#endif // SPI_DRIVER_H
