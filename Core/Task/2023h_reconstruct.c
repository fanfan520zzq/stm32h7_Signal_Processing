//
// Rebuild A' and B' from the measured 2023H signal parameters.
//

#include "2023h_reconstruct.h"
#include "DDS.h"
#include "dac.h"
#include <math.h>
#include <stddef.h>

#define RECON_PI              3.14159265f
#define RECON_SWEEP_FS_HZ     2400000.0f
#define RECON_TRI_FUND_RATIO  (8.0f / (RECON_PI * RECON_PI))
#define RECON_MAX_DAC_VPP_MV  2000.0f

static uint8_t Reconstruct_ToDdsWaveType(WaveType type)
{
    switch (type) {
        case SIG_SINE:
            return 0u;
        case SIG_TRIANGLE:
            return 2u;
        default:
            return 0u;
    }
}

static float Reconstruct_ToWaveVpp(const SignalInfo *sig)
{
    if (sig->type == SIG_TRIANGLE) {
        return sig->amp / RECON_TRI_FUND_RATIO;
    }
    return sig->amp;
}

static uint16_t Reconstruct_ToMillivolts(float vpp)
{
    float mv = vpp * 1000.0f;

    if (mv < 0.0f) {
        mv = 0.0f;
    }
    if (mv > RECON_MAX_DAC_VPP_MV) {
        mv = RECON_MAX_DAC_VPP_MV;
    }

    return (uint16_t)(mv + 0.5f);
}

static float Reconstruct_ToDdsPhase(const SignalInfo *sig)
{
    const float goertzel_sample_delay = 2.0f * RECON_PI * (float)sig->freq / RECON_SWEEP_FS_HZ;
    float phase = sig->phase + (RECON_PI * 0.5f) + goertzel_sample_delay;

    while (phase > RECON_PI) {
        phase -= 2.0f * RECON_PI;
    }
    while (phase < -RECON_PI) {
        phase += 2.0f * RECON_PI;
    }

    return phase;
}

static void Reconstruct_OutputOne(uint8_t channel, const SignalInfo *sig)
{
    const uint16_t vpp_mv = Reconstruct_ToMillivolts(Reconstruct_ToWaveVpp(sig));
    const uint8_t dds_type = Reconstruct_ToDdsWaveType(sig->type);
    const float dds_phase = Reconstruct_ToDdsPhase(sig);

    if (channel == 1u) {
        DDS1_Update_DATA(sig->freq, vpp_mv, dds_type);
        DDS1_Add_Phase((int32_t)(dds_phase / (2.0f * RECON_PI) * 4294967296.0f));
    } else {
        DDS2_Update_DATA(sig->freq, vpp_mv, dds_type);
        // NOTE: DDS2_Add_Phase doesn't exist yet, but since this is dead code we can just comment it or add it
        extern volatile uint32_t dds2_phase_acc;
        dds2_phase_acc += (int32_t)(dds_phase / (2.0f * RECON_PI) * 4294967296.0f);
    }
}

void Reconstruct2023H_Output(const SignalSeparationResult *result)
{
    HAL_TIM_Base_Start(&htim6);
    if (result == NULL || result->valid_count <= 0) {
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
        return;
    }

    Reconstruct_OutputOne(1u, &result->sig1);

    if (result->valid_count >= 2) {
        Reconstruct_OutputOne(2u, &result->sig2);
    } else {
        HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
    }
}

SignalSeparationResult Execute_Separation_And_Reconstruct2023H(void)
{
    SignalSeparationResult result = Execute_Signal_Separation();
    Reconstruct2023H_Output(&result);
    return result;
}
