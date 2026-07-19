#include "dpll_b_mode.h"
#include <stddef.h>

int32_t DPLL_B_PhaseDegreesToU32(uint16_t degrees, uint32_t *phase_u32) {
    if (phase_u32 == NULL || degrees > 180U || (degrees % 5U) != 0U) return -1;
    *phase_u32 = (uint32_t)((((uint64_t)degrees << 32U) + 180U) / 360U);
    return 0;
}

int32_t DPLL_B_CommonPpmFTW(uint32_t nominal_ftw_a,
                            uint32_t nominal_ftw_b,
                            uint32_t active_ftw_a,
                            uint32_t *active_ftw_b) {
    if (nominal_ftw_a == 0U || nominal_ftw_b == 0U || active_ftw_b == NULL) return -1;
    int64_t delta_a = (int64_t)active_ftw_a - (int64_t)nominal_ftw_a;
    int64_t numerator = delta_a * (int64_t)nominal_ftw_b;
    int64_t scaled_delta = numerator >= 0
        ? (numerator + (int64_t)nominal_ftw_a / 2) / (int64_t)nominal_ftw_a
        : (numerator - (int64_t)nominal_ftw_a / 2) / (int64_t)nominal_ftw_a;
    int64_t result = (int64_t)nominal_ftw_b + scaled_delta;
    if (result <= 0 || result > (int64_t)UINT32_MAX) return -1;
    *active_ftw_b = (uint32_t)result;
    return 0;
}

int32_t DPLL_B_ValidateDerived(uint32_t input_a_hz, uint32_t input_b_hz,
                               uint8_t ratio_n, uint16_t phase_degrees) {
    uint32_t ignored;
    if (input_a_hz == 0U || ratio_n == 0U ||
        input_b_hz != input_a_hz * (uint32_t)ratio_n) return -1;
    return DPLL_B_PhaseDegreesToU32(phase_degrees, &ignored);
}
