#ifndef RECON_DDS_H
#define RECON_DDS_H

#include "recon_types.h"
#include <stdint.h>

extern volatile uint8_t g_recon_dds_active;
extern volatile uint32_t g_recon_dds_phase_acc;
extern volatile uint32_t g_recon_dds_ftw;

void recon_dds_init(void);
void recon_dds_load_lut(const uint16_t *lut, uint32_t len);
void recon_dds_set_freq(float freq_hz);
void recon_dds_update_ftw(uint32_t ftw);
uint32_t recon_dds_freq_to_ftw(double freq_hz);
double recon_dds_ftw_to_freq(uint32_t ftw);
void recon_dds_start(float freq_hz);
void recon_dds_start_phase(float freq_hz, float phase_rad);
void recon_dds_stop(void);
void recon_dds_fill(uint16_t *dst, uint32_t len);

#endif
