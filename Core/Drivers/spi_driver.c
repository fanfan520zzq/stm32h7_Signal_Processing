#include "spi_driver.h"
#include "spi.h"
#include "module_state.h"

void SPI_Driver_Init(void) {
    // CubeMX MX_SPI2_Init() already called in main.c
    // Additional initialization can go here
}

int32_t SPI_Driver_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    if (HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, Size, Timeout) == HAL_OK) {
        return ERR_OK;
    }
    return -1; // Generic error, can expand to more specific codes if needed
}
