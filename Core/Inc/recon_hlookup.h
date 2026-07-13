#ifndef RECON_HLOOKUP_H
#define RECON_HLOOKUP_H

#include "recon_types.h"

extern uint8_t g_recon_bypass_h;

int recon_hlookup(float freq_hz, ReconHPoint *out);

#endif
