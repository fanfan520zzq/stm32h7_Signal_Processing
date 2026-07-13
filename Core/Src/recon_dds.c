#include "recon_dds.h"
#include "DDS.h"
#include "dac.h"
#include <string.h>

volatile uint8_t g_recon_dds_active = 0;
volatile uint32_t g_recon_dds_phase_acc = 0;
volatile uint32_t g_recon_dds_ftw = 0;

static uint16_t s_recon_lut[RECON_TABLE_LEN];

void recon_dds_init(void)
{
    for (uint32_t i = 0; i < RECON_TABLE_LEN; i++) {
        s_recon_lut[i] = 1241u;
    }
    g_recon_dds_phase_acc = 0u;
    g_recon_dds_ftw = 0u;
}

void recon_dds_load_lut(const uint16_t *lut, uint32_t len)
{
    if (lut == 0 || len == 0u) {
        return;
    }
    if (len > RECON_TABLE_LEN) {
        len = RECON_TABLE_LEN;
    }

    __disable_irq();
    for (uint32_t i = 0; i < len; i++) {
        s_recon_lut[i] = lut[i];
    }
    if (len < RECON_TABLE_LEN) {
        for (uint32_t i = len; i < RECON_TABLE_LEN; i++) {
            s_recon_lut[i] = lut[len - 1u];
        }
    }
    __enable_irq();
}

uint32_t recon_dds_freq_to_ftw(double freq_hz)
{
    if (freq_hz < 0.0) {
        freq_hz = 0.0;
    }
    return (uint32_t)(freq_hz / (double)RECON_DAC_FS_HZ * 4294967296.0);
}

double recon_dds_ftw_to_freq(uint32_t ftw)
{
    return (double)ftw * (double)RECON_DAC_FS_HZ / 4294967296.0;
}

void recon_dds_set_freq(float freq_hz)
{
    g_recon_dds_ftw = recon_dds_freq_to_ftw((double)freq_hz);
}

void recon_dds_update_ftw(uint32_t ftw)
{
    g_recon_dds_ftw = ftw;
}

void recon_dds_fill(uint16_t *dst, uint32_t len)
{
    if (dst == 0) {
        return;
    }

    for (uint32_t i = 0; i < len; i++) {
        g_recon_dds_phase_acc += g_recon_dds_ftw;
        dst[i] = s_recon_lut[g_recon_dds_phase_acc >> 22];
    }
}

void recon_dds_start_phase(float freq_hz, float phase_rad)
{
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_TIM_Base_Start(&htim6);
    recon_dds_set_freq(freq_hz);
    while (phase_rad < 0.0f) phase_rad += 2.0f * RECON_PI;
    while (phase_rad >= 2.0f * RECON_PI) phase_rad -= 2.0f * RECON_PI;
    g_recon_dds_phase_acc = (uint32_t)((double)phase_rad /
                                       (2.0 * (double)RECON_PI) * 4294967296.0);
    g_recon_dds_active = 1u;
    recon_dds_fill(Buffer1, RECON_TABLE_LEN);
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)Buffer1, RECON_TABLE_LEN, DAC_ALIGN_12B_R);
}

void recon_dds_start(float freq_hz)
{
    recon_dds_start_phase(freq_hz, 0.0f);
}

void recon_dds_stop(void)
{
    g_recon_dds_active = 0u;
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}
