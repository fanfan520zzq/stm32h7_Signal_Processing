#include "fpga_spi_protocol.h"
#include "spi_driver.h"
#include "module_state.h"

uint8_t FPGA_Protocol_CRC8(const uint8_t *data, uint16_t length) {
    uint8_t crc = 0U;
    for (uint16_t i = 0U; i < length; ++i) {
        uint8_t byte = data[i];
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            uint8_t feedback = (uint8_t)(((crc >> 7U) ^ (byte >> (7U - bit))) & 1U);
            crc = (uint8_t)(crc << 1U);
            if (feedback != 0U) crc ^= 0x07U;
        }
    }
    return crc;
}

int32_t FPGA_Protocol_Write16(uint8_t address, uint16_t value) {
    uint8_t tx[4];
    uint8_t rx[4] = {0U};
    tx[0] = (uint8_t)((address & 0x7FU) | 0x80U);
    tx[1] = (uint8_t)(value >> 8U);
    tx[2] = (uint8_t)value;
    tx[3] = FPGA_Protocol_CRC8(tx, 3U);
    return SPI_Driver_TransferFrame(tx, rx);
}

int32_t FPGA_Protocol_Read16(uint8_t address, uint16_t *value) {
    uint8_t tx[4];
    uint8_t rx[4] = {0U};
    int32_t result;
    if (value == NULL) return ERR_PARAM;
    tx[0] = (uint8_t)(address & 0x7FU);
    tx[1] = 0U;
    tx[2] = 0U;
    tx[3] = FPGA_Protocol_CRC8(tx, 3U);
    result = SPI_Driver_TransferFrame(tx, rx);
    if (result != ERR_OK) return result;
    if (FPGA_Protocol_CRC8(rx, 3U) != rx[3]) return ERR_CRC;
    *value = (uint16_t)(((uint16_t)rx[1] << 8U) | rx[2]);
    return ERR_OK;
}

int32_t FPGA_Protocol_Write32(uint8_t low_address, uint32_t value) {
    int32_t result = FPGA_Protocol_Write16(low_address, (uint16_t)value);
    if (result != ERR_OK) return result;
    return FPGA_Protocol_Write16((uint8_t)(low_address + 1U), (uint16_t)(value >> 16U));
}

int32_t FPGA_Protocol_Read32(uint8_t low_address, uint32_t *value) {
    uint16_t low = 0U, high = 0U;
    int32_t result;
    if (value == NULL) return ERR_PARAM;
    result = FPGA_Protocol_Read16(low_address, &low);
    if (result != ERR_OK) return result;
    result = FPGA_Protocol_Read16((uint8_t)(low_address + 1U), &high);
    if (result != ERR_OK) return result;
    *value = ((uint32_t)high << 16U) | low;
    return ERR_OK;
}

int32_t FPGA_Protocol_Read64(uint8_t low_address, uint64_t *value) {
    uint64_t assembled = 0U;
    if (value == NULL) return ERR_PARAM;
    for (uint8_t word = 0U; word < 4U; ++word) {
        uint16_t part = 0U;
        int32_t result = FPGA_Protocol_Read16((uint8_t)(low_address + word), &part);
        if (result != ERR_OK) return result;
        assembled |= ((uint64_t)part << (16U * word));
    }
    *value = assembled;
    return ERR_OK;
}
