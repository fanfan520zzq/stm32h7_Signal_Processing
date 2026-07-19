#ifndef FPGA_SPI_PROTOCOL_H
#define FPGA_SPI_PROTOCOL_H

#include <stdint.h>

#define FPGA_REG_ID                    0x00U
#define FPGA_REG_TEST                  0x01U
#define FPGA_REG_CTRL_EN               0x02U
#define FPGA_REG_CH1_FREQ_L            0x03U
#define FPGA_REG_CH1_FREQ_H            0x04U
#define FPGA_REG_CH2_FREQ_L            0x05U
#define FPGA_REG_CH2_FREQ_H            0x06U
#define FPGA_REG_CH1_WAVE              0x07U
#define FPGA_REG_CH2_WAVE              0x08U
#define FPGA_REG_COMMIT                0x0FU
#define FPGA_REG_SYNC                  0x10U
#define FPGA_REG_SEQ                   0x11U
#define FPGA_REG_LATCH_CNT_L           0x12U
#define FPGA_REG_LATCH_CNT_H           0x13U
#define FPGA_REG_LIVE_CNT_L            0x14U
#define FPGA_REG_LIVE_CNT_H            0x15U
#define FPGA_REG_PROTOCOL_VERSION      0x16U
#define FPGA_REG_CAPABILITIES          0x17U
#define FPGA_REG_STATUS                0x18U
#define FPGA_REG_SNAP_ARM              0x19U
#define FPGA_REG_SNAP_SEQ              0x1AU
#define FPGA_REG_ERROR_COUNT           0x1BU
#define FPGA_REG_BUILD_ID              0x1CU
#define FPGA_REG_SNAP_SAMPLE_COUNTER   0x20U
#define FPGA_REG_SNAP_APPLY_COUNTER    0x24U
#define FPGA_REG_SNAP_PHASE_A          0x28U
#define FPGA_REG_SNAP_PHASE_B          0x2AU
#define FPGA_REG_SNAP_ACTIVE_FTW_A     0x2CU
#define FPGA_REG_SNAP_ACTIVE_FTW_B     0x2EU
#define FPGA_REG_SHADOW_FTW_A          0x30U
#define FPGA_REG_SHADOW_FTW_B          0x32U
#define FPGA_REG_PHASE_OFFSET_B        0x34U
#define FPGA_REG_PHASE_MODE            0x36U

#define FPGA_CAP_SNAPSHOT              (1U << 0)
#define FPGA_CAP_RAW_FTW               (1U << 1)
#define FPGA_CAP_ATOMIC_COMMIT         (1U << 2)
#define FPGA_CAP_DERIVED_B             (1U << 3)
#define FPGA_CAP_COUNTER64             (1U << 4)
#define FPGA_CAP_STAGE07_REQUIRED      0x001FU

#define FPGA_STATUS_SNAP_ARMED         (1U << 0)
#define FPGA_STATUS_SNAP_VALID         (1U << 1)
#define FPGA_STATUS_CTRL_EN            (1U << 2)
#define FPGA_STATUS_RAW_FTW_ACTIVE     (1U << 3)
#define FPGA_STATUS_DERIVED_B_ACTIVE   (1U << 4)

#define FPGA_PHASE_MODE_RATIO_MASK     0x00FFU
#define FPGA_PHASE_MODE_DERIVED_B      (1U << 8)
#define FPGA_PHASE_MODE_RAW_FTW        (1U << 9)

#define FPGA_WAVE_SINE                 0x00U
#define FPGA_WAVE_SQUARE               0x01U
#define FPGA_WAVE_TRIANGLE             0x02U
#define FPGA_WAVE_DC                   0x03U

uint8_t FPGA_Protocol_CRC8(const uint8_t *data, uint16_t length);
int32_t FPGA_Protocol_Write16(uint8_t address, uint16_t value);
int32_t FPGA_Protocol_Read16(uint8_t address, uint16_t *value);
int32_t FPGA_Protocol_Write32(uint8_t low_address, uint32_t value);
int32_t FPGA_Protocol_Read32(uint8_t low_address, uint32_t *value);
int32_t FPGA_Protocol_Read64(uint8_t low_address, uint64_t *value);

#endif /* FPGA_SPI_PROTOCOL_H */
