#ifndef DPLL_B_MODE_H
#define DPLL_B_MODE_H

#include <stdint.h>

typedef enum {
    DPLL_B_COMMON_PPM = 0,
    DPLL_B_DERIVED_INTEGER = 1
} DPLL_BMode_t;

int32_t DPLL_B_PhaseDegreesToU32(uint16_t degrees, uint32_t *phase_u32);
int32_t DPLL_B_CommonPpmFTW(uint32_t nominal_ftw_a,
                            uint32_t nominal_ftw_b,
                            uint32_t active_ftw_a,
                            uint32_t *active_ftw_b);
int32_t DPLL_B_ValidateDerived(uint32_t input_a_hz, uint32_t input_b_hz,
                               uint8_t ratio_n, uint16_t phase_degrees);

#endif /* DPLL_B_MODE_H */
