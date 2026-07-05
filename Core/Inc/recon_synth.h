#ifndef RECON_SYNTH_H
#define RECON_SYNTH_H

#include "recon_types.h"
#include <stdint.h>

int recon_synth_build_lut(const ReconAnalysis *analysis, uint16_t *lut, uint32_t lut_len,
                          float bias_v, float max_vpp_v, uint8_t *used_harmonics);

#endif
