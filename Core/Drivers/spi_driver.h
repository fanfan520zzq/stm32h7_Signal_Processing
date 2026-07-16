#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "main.h"

// Initialize SPI driver (if needed beyond CubeMX)
void SPI_Driver_Init(void);

// Blocking Transmit and Receive for SPI2
// Returns 0 on success, <0 on error
int32_t SPI_Driver_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

#endif // SPI_DRIVER_H
