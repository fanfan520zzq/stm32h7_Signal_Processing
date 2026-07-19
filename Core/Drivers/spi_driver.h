#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "main.h"

// Initialize SPI driver (if needed beyond CubeMX)
void SPI_Driver_Init(void);

// Basic blocking Transmit and Receive for SPI2
int32_t SPI_Driver_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);

// High level FPGA Register Access
int32_t SPI_Driver_WriteReg(uint8_t addr, uint16_t data);
int32_t SPI_Driver_ReadReg(uint8_t addr, uint16_t *data);

#endif // SPI_DRIVER_H
