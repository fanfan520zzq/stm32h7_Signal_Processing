#include "spi_driver.h"
#include "spi.h"
#include "module_state.h"

void SPI_Driver_Init(void) {
    // CubeMX MX_SPI2_Init() already called in main.c
    // Ensure CS pin (PF9) is high
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
}

int32_t SPI_Driver_TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout) {
    if (HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, Size, Timeout) == HAL_OK) {
        return ERR_OK;
    }
    return -1; // Generic error
}

static uint8_t CalcCRC8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            uint8_t bit = (b >> (7 - j)) & 1;
            uint8_t crc_msb = (crc >> 7) & 1;
            crc = (crc << 1) & 0xFF;
            if (crc_msb ^ bit) {
                crc ^= 0x07;
            }
        }
    }
    return crc;
}

int32_t SPI_Driver_WriteReg(uint8_t addr, uint16_t data) {
    uint8_t tx_buf[4];
    uint8_t rx_buf[4];
    tx_buf[0] = (addr & 0x7F) | 0x80; // bit 7 = 1 (Write)
    tx_buf[1] = (data >> 8) & 0xFF;
    tx_buf[2] = data & 0xFF;
    tx_buf[3] = CalcCRC8(tx_buf, 3);
    
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); // CS Low
    int32_t res = SPI_Driver_TransmitReceive(tx_buf, rx_buf, 4, 100);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);   // CS High
    
    // Add delay to guarantee CS high time is > 40ns for FPGA (which runs at 25MHz)
    for (volatile int i = 0; i < 200; i++) {}
    
    return res;
}

int32_t SPI_Driver_ReadReg(uint8_t addr, uint16_t *data) {
    uint8_t tx_buf[4];
    uint8_t rx_buf[4] = {0};
    tx_buf[0] = addr & 0x7F; // bit 7 = 0 (Read)
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = CalcCRC8(tx_buf, 3);
    
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET); // CS Low
    int32_t res = SPI_Driver_TransmitReceive(tx_buf, rx_buf, 4, 100);
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);   // CS High
    
    if (res == ERR_OK) {
        // Validate RX CRC
        uint8_t expected_crc = CalcCRC8(rx_buf, 3);
        if (expected_crc != rx_buf[3]) {
            return ERR_CRC;
        }
        *data = (rx_buf[1] << 8) | rx_buf[2];
    }
    return res;
}
