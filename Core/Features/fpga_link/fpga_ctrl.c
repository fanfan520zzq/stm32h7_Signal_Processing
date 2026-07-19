#include "fpga_ctrl.h"
#include "fpga_spi_protocol.h"
#include "spi_driver.h"
#include "module_state.h"
#include <stdio.h>

volatile uint32_t fpga_spi_err_count = 0;

// Wrapper that counts every failed SPI transaction
static int32_t fpga_wr(uint8_t addr, uint16_t data) {
    int32_t r = SPI_Driver_WriteReg(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

static int32_t fpga_rd(uint8_t addr, uint16_t *data) {
    int32_t r = SPI_Driver_ReadReg(addr, data);
    if (r != ERR_OK) fpga_spi_err_count++;
    return r;
}

void FPGA_Ctrl_Init(void) {
    SPI_Driver_Init();

    int32_t wres = fpga_wr(FPGA_REG_CTRL_EN, 1);

    // Read back to prove the link and the override actually took effect
    uint16_t id = 0, ctrl = 0;
    int32_t r_id  = fpga_rd(FPGA_REG_ID, &id);
    int32_t r_ctl = fpga_rd(FPGA_REG_CTRL_EN, &ctrl);

    printf("LOG:INFO FPGA Init: ID=0x%04X(res=%ld) CTRL_EN=%u(res=%ld) wr_res=%ld err=%lu\r\n",
           id, (long)r_id, ctrl, (long)r_ctl, (long)wres,
           (unsigned long)fpga_spi_err_count);
}

int32_t FPGA_Ctrl_SetFreq(uint8_t ch, uint32_t freq_hz) {
    int32_t errs = 0;
    if (ch == 1) {
        errs += (fpga_wr(FPGA_REG_CH1_FREQ_L, freq_hz & 0xFFFF) != ERR_OK);
        errs += (fpga_wr(FPGA_REG_CH1_FREQ_H, (freq_hz >> 16) & 0xFFFF) != ERR_OK);
    } else {
        errs += (fpga_wr(FPGA_REG_CH2_FREQ_L, freq_hz & 0xFFFF) != ERR_OK);
        errs += (fpga_wr(FPGA_REG_CH2_FREQ_H, (freq_hz >> 16) & 0xFFFF) != ERR_OK);
    }
    return -errs;
}

int32_t FPGA_Ctrl_SetWave(uint8_t ch, uint8_t wave_type, uint8_t vpp) {
    uint16_t val = ((uint16_t)vpp << 8) | (wave_type & 0x03);
    int32_t r = (ch == 1) ? fpga_wr(FPGA_REG_CH1_WAVE, val)
                          : fpga_wr(FPGA_REG_CH2_WAVE, val);
    return (r == ERR_OK) ? 0 : -1;
}

void FPGA_Ctrl_Commit(void) {
    fpga_wr(FPGA_REG_COMMIT, 1);
}

void FPGA_Ctrl_Sync(void) {
    fpga_wr(FPGA_REG_SYNC, 1);
}

void FPGA_Ctrl_UpdateFromAlgorithm(uint8_t ch, uint8_t wave_type, uint32_t freq_hz) {
    // Default Vpp = 15 (1.5V, safely above the 1Vpp requirement, no DAC wrap)
    FPGA_Ctrl_SetWave(ch, wave_type, 15);
    FPGA_Ctrl_SetFreq(ch, freq_hz);
    FPGA_Ctrl_Commit();
}

int32_t FPGA_Ctrl_ApplyResult(const SignalSeparationResult *res) {
    if (!res || res->valid_count <= 0) return -1;

    int32_t errs = 0;
    uint32_t applied_freq[2] = {0, 0};

    // CTRL_EN 每次重新断言：FPGA 掉电/未配置会丢配置，单次 boot 写入不可靠
    errs += (fpga_wr(FPGA_REG_CTRL_EN, 1) != ERR_OK);

    // Separate_Signals 按 20k->100k 升序扫描：sig1 必为低频(A)->CH1，sig2 高频(B)->CH2
    for (int i = 0; i < res->valid_count && i < 2; i++) {
        const SignalInfo *s = (i == 0) ? &res->sig1 : &res->sig2;
        uint8_t ch = (uint8_t)(i + 1);

        if (s->type == WAVE_UNKNOWN || s->freq == 0) continue;

        uint8_t wave = (s->type == SIG_TRIANGLE) ? FPGA_WAVE_TRIANGLE : FPGA_WAVE_SINE;

        // vpp_10x = Vpp * 10，钳到 [10, 115]：保证 >=1Vpp 且 amp_adj 不溢出
        uint32_t vpp10 = (uint32_t)(s->amp * 10.0f + 0.5f);
        if (vpp10 < 10)  vpp10 = 10;
        if (vpp10 > 115) vpp10 = 115;

        errs += (FPGA_Ctrl_SetWave(ch, wave, (uint8_t)vpp10) != 0);
        errs += (FPGA_Ctrl_SetFreq(ch, s->freq) != 0);
        applied_freq[i] = s->freq;
    }

    // 两路 shadow 写完，一次 COMMIT 同边界生效
    FPGA_Ctrl_Commit();

    // 读回验证：shadow 值 + 配置序号
    uint16_t seq = 0, f1l = 0, f1h = 0, f2l = 0, f2h = 0;
    errs += (fpga_rd(FPGA_REG_SEQ, &seq) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH1_FREQ_L, &f1l) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH1_FREQ_H, &f1h) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH2_FREQ_L, &f2l) != ERR_OK);
    errs += (fpga_rd(FPGA_REG_CH2_FREQ_H, &f2h) != ERR_OK);

    uint32_t rb1 = ((uint32_t)f1h << 16) | f1l;
    uint32_t rb2 = ((uint32_t)f2h << 16) | f2l;

    printf("LOG:INFO FPGA apply: ch1=%luHz ch2=%luHz rb_ch1=%luHz rb_ch2=%luHz seq=%u errs=%ld crcum=%lu\r\n",
           (unsigned long)applied_freq[0], (unsigned long)applied_freq[1],
           (unsigned long)rb1, (unsigned long)rb2,
           seq, (long)errs, (unsigned long)fpga_spi_err_count);

    return errs;
}

void FPGA_Ctrl_PrintStatus(void) {
    uint16_t id = 0, ctrl = 0, seq = 0;
    int32_t r1 = fpga_rd(FPGA_REG_ID, &id);
    int32_t r2 = fpga_rd(FPGA_REG_CTRL_EN, &ctrl);
    int32_t r3 = fpga_rd(FPGA_REG_SEQ, &seq);
    printf("ACK:FPGA_STATUS id=0x%04X(%ld) ctrl_en=%u(%ld) seq=%u(%ld) err=%lu\r\n",
           id, (long)r1, ctrl, (long)r2, seq, (long)r3,
           (unsigned long)fpga_spi_err_count);
}
