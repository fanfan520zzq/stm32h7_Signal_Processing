//
// Created by Lenovo on 2026/5/1.
//

#include "Measure.h"
#include "DDS.h"
#include "ADCTask.h"

#define RS_OHM        500.0f
#define ADC_TO_VOLT   (3.3f / 65535.0f)

float Measure_Input_Resistance(void)
{
    DDS2_Update_DATA(1000, 200, 0);

    uint16_t vpp1, vpp2;
    ADC1_Measure_Sync(&vpp1, &vpp2);

    float diff_vpp = (float)(vpp1 > vpp2 ? vpp1 - vpp2 : vpp2 - vpp1);

    if (diff_vpp < 1.0f) return 0.0f;

    float i_ma = diff_vpp * ADC_TO_VOLT / RS_OHM;

    float larger_vpp = (float)(vpp1 > vpp2 ? vpp1 : vpp2);

    return larger_vpp * ADC_TO_VOLT / i_ma;
}