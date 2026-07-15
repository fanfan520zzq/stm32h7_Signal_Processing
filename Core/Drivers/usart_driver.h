#ifndef USART_DRIVER_H
#define USART_DRIVER_H

#include "main.h"

// Initialize driver for a specific UART (starts DMA/IT Rx)
void USART_Driver_Init(UART_HandleTypeDef *huart);

// Read one byte from the ring buffer of a specific UART
// Returns 1 if successful, 0 if buffer is empty
uint8_t USART_Driver_ReadByte(UART_HandleTypeDef *huart, uint8_t *byte);

// Write bytes to a specific UART
void USART_Driver_WriteBytes(UART_HandleTypeDef *huart, const uint8_t *data, uint16_t len);

// Must be called inside HAL_UARTEx_RxEventCallback
void USART_Driver_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif // USART_DRIVER_H
