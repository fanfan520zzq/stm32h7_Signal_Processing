#ifndef FPGA_SPI_PROTOCOL_H
#define FPGA_SPI_PROTOCOL_H

// ---------------------------------------------------------
// FPGA SPI Register Map
// ---------------------------------------------------------
#define FPGA_REG_ID           0x00
#define FPGA_REG_TEST         0x01
#define FPGA_REG_CTRL_EN      0x02
#define FPGA_REG_CH1_FREQ_L   0x03
#define FPGA_REG_CH1_FREQ_H   0x04
#define FPGA_REG_CH2_FREQ_L   0x05
#define FPGA_REG_CH2_FREQ_H   0x06
#define FPGA_REG_CH1_WAVE     0x07
#define FPGA_REG_CH2_WAVE     0x08
#define FPGA_REG_COMMIT       0x0F
#define FPGA_REG_SYNC         0x10
#define FPGA_REG_SEQ          0x11
#define FPGA_REG_LATCH_CNT_L  0x12
#define FPGA_REG_LATCH_CNT_H  0x13
#define FPGA_REG_LIVE_CNT_L   0x14
#define FPGA_REG_LIVE_CNT_H   0x15

// Waveform definitions matching FPGA mapping
#define FPGA_WAVE_SINE        0x00
#define FPGA_WAVE_SQUARE      0x01
#define FPGA_WAVE_TRIANGLE    0x02
#define FPGA_WAVE_DC          0x03

#endif // FPGA_SPI_PROTOCOL_H
