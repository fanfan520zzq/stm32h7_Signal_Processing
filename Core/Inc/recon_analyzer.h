#ifndef RECON_ANALYZER_H
#define RECON_ANALYZER_H

#include "recon_types.h"
#include <stdint.h>

int recon_analyze_block(const uint16_t *buf, uint32_t len, float fs_hz, ReconAnalysis *out);
void recon_print_analysis(const ReconAnalysis *analysis);

#endif
