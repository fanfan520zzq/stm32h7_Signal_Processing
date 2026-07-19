#include "dpll_b_mode.h"
#include <stdint.h>
#include <stdio.h>

int main(void) {
    unsigned failures = 0U;
    uint32_t phase = 0U;
    if (DPLL_B_PhaseDegreesToU32(0U, &phase) != 0 || phase != 0U) failures++;
    if (DPLL_B_PhaseDegreesToU32(90U, &phase) != 0 || phase != 0x40000000U) failures++;
    if (DPLL_B_PhaseDegreesToU32(180U, &phase) != 0 || phase != 0x80000000U) failures++;
    if (DPLL_B_PhaseDegreesToU32(3U, &phase) == 0) failures++;
    if (DPLL_B_PhaseDegreesToU32(185U, &phase) == 0) failures++;

    uint32_t ftw_b = 0U;
    if (DPLL_B_CommonPpmFTW(1000000U, 1666667U, 1000100U, &ftw_b) != 0 ||
        ftw_b != 1666834U) failures++;
    if (DPLL_B_CommonPpmFTW(1000000U, 1666667U, 999900U, &ftw_b) != 0 ||
        ftw_b != 1666500U) failures++;
    if (DPLL_B_ValidateDerived(30000U, 60000U, 2U, 45U) != 0) failures++;
    if (DPLL_B_ValidateDerived(30000U, 50000U, 2U, 45U) == 0) failures++;
    if (DPLL_B_ValidateDerived(30000U, 60000U, 2U, 47U) == 0) failures++;

    if (failures != 0U) {
        printf("DPLL_B_MODE_FAIL failures=%u\n", failures);
        return 1;
    }
    printf("DPLL_B_MODE_PASS phase_0_180_step_5=1 common_ppm=1 derived_ratio=1\n");
    return 0;
}
