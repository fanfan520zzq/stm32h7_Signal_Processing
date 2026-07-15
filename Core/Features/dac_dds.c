#include "dac_dds.h"
#include "clock_service.h"
#include <math.h>

#define DDS_DMA_BUF_SIZE 2000
#define DDS_LUT_SIZE     4096

// Place DMA buffer in non-cached D2 RAM if necessary, or clean cache manually
uint16_t dds_dma_buf[DDS_DMA_BUF_SIZE] __attribute__((section(".dma_buffer")));

static uint16_t sin_lut[DDS_LUT_SIZE];

// DDS State
static volatile uint32_t dds_phase_acc = 0;
static volatile uint32_t dds_ftw = 0;
static volatile uint8_t  dds_wave_type = DDS_WAVE_SINE;
static volatile uint32_t dds_duty_thresh = 0; // 0 to 0xFFFFFFFF (for 0-100%)
static volatile float    dds_scale = 1.0f;
static volatile uint16_t dds_bias_code = 2048;

static uint32_t dds_update_rate = 1000000; // 1MHz DAC timer

void DDS_Init(void)
{
    // Generate Sine LUT (0 to 1 amplitude, centered at 0, ranging -0.5 to 0.5)
    for(int i = 0; i < DDS_LUT_SIZE; i++) {
        float rad = (2.0f * 3.1415926535f * i) / DDS_LUT_SIZE;
        sin_lut[i] = (uint16_t)((sinf(rad) + 1.0f) * 2047.5f); // 0 to 4095
    }
    
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}

void DDS_ConfigTrigger(uint32_t trigger_source)
{
    DAC_ChannelConfTypeDef sConfig = {0};
    sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    sConfig.DAC_Trigger = trigger_source;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
    sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    
    HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1);
}

void DDS_SetParam(uint8_t waveType, uint32_t freq_hz, uint16_t vpp_mv, uint16_t bias_mv, uint8_t duty_cycle)
{
    dds_wave_type = waveType;
    
    // FTW = freq * (2^32) / update_rate
    dds_ftw = (uint32_t)(((uint64_t)freq_hz << 32) / dds_update_rate);
    
    // Scale and bias
    // 3.3V = 4095
    dds_scale = (float)vpp_mv / 3300.0f;
    dds_bias_code = (uint16_t)(((float)bias_mv / 3300.0f) * 4095.0f);
    
    // Duty cycle threshold (for square wave)
    if (duty_cycle > 100) duty_cycle = 50;
    dds_duty_thresh = (uint32_t)(((uint64_t)duty_cycle * 0xFFFFFFFFULL) / 100ULL);
}

void DDS_Start(void)
{
    // Ensure DAC frequency is set to 1MHz via SI5351
    Clock_Service_SetDACFreq(1, CLOCK_SRC_EXTERNAL_SI5351, 1000000, &dds_update_rate);
    
    // Prime the buffer with 0 to prevent glitch on start
    for (int i = 0; i < DDS_DMA_BUF_SIZE; i++) {
        dds_dma_buf[i] = dds_bias_code;
    }
    
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dds_dma_buf, DDS_DMA_BUF_SIZE, DAC_ALIGN_12B_R);
}

void DDS_Stop(void)
{
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}

// Fill half of the buffer
static void DDS_FillBuffer(uint16_t* buf, uint32_t length)
{
    uint32_t local_acc = dds_phase_acc;
    uint32_t local_ftw = dds_ftw;
    uint8_t  local_wave = dds_wave_type;
    uint32_t local_thresh = dds_duty_thresh;
    float    local_scale = dds_scale;
    int32_t  local_bias = dds_bias_code;
    
    int32_t half_amp = (int32_t)(2048.0f * local_scale);
    
    for(uint32_t i = 0; i < length; i++) {
        local_acc += local_ftw;
        int32_t val = 0;
        
        switch (local_wave) {
            case DDS_WAVE_SINE: {
                uint32_t idx = local_acc >> 20; // top 12 bits
                int32_t lut_val = (int32_t)sin_lut[idx] - 2048; // -2048 to +2047
                val = (int32_t)((float)lut_val * local_scale);
                break;
            }
            case DDS_WAVE_SQUARE: {
                val = (local_acc < local_thresh) ? half_amp : -half_amp;
                break;
            }
            case DDS_WAVE_TRIANGLE: {
                // local_acc ranges 0 to 2^32-1
                // We want a triangle from -half_amp to +half_amp
                // 0 to 2^31: ramps up from -half_amp to +half_amp
                // 2^31 to 2^32: ramps down
                if (local_acc < 0x80000000) {
                    val = -half_amp + (int32_t)(((uint64_t)local_acc * (2 * half_amp)) >> 31);
                } else {
                    val = half_amp - (int32_t)(((uint64_t)(local_acc - 0x80000000) * (2 * half_amp)) >> 31);
                }
                break;
            }
        }
        
        val += local_bias;
        if (val < 0) val = 0;
        if (val > 4095) val = 4095;
        
        buf[i] = (uint16_t)val;
    }
    dds_phase_acc = local_acc;
}

// DMA Callbacks
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    DDS_FillBuffer(&dds_dma_buf[0], DDS_DMA_BUF_SIZE / 2);
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    DDS_FillBuffer(&dds_dma_buf[DDS_DMA_BUF_SIZE / 2], DDS_DMA_BUF_SIZE / 2);
}
