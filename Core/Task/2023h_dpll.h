#ifndef ITVM_DDS_2023H_DPLL_H
#define ITVM_DDS_2023H_DPLL_H

#include "main.h"
#include "2023h_signal_seperate.h"

void DPLL2023H_Init(const SignalSeparationResult *sep);
void DPLL2023H_Update(const SignalSeparationResult *sep);
uint8_t DPLL2023H_IsLostLock(double max_err_rad, uint32_t max_count);
void DPLL2023H_SetPhaseOffset(uint8_t ch, float offset_deg);
void DPLL2023H_SetMutualPhase(uint8_t enable, float offset_deg);
void DPLL2023H_ForcePhaseAlign(float offset_deg);

#endif //ITVM_DDS_2023H_DPLL_H
