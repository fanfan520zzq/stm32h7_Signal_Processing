#include "DDS.h"

#define DAC_BIAS_1_3V      1613.18f  /* 1.3 V DC = 4096 * 1.3 / 3.3 */
#define DAC_AMP_0_5V       620.606f  /* 0.5 V amplitude = 4096 * 0.5 / 3.3 */
#define DDS_PI             3.14159265f

// 双缓冲 DMA，大小必须是偶数
#define DDS_BUF_SIZE       1000

uint16_t Buffer1[DDS_BUF_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
uint16_t Buffer2[DDS_BUF_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));

volatile uint32_t dds1_phase_acc = 0;
volatile uint32_t dds1_ftw = 0;
volatile uint16_t dds1_vpp = 1000;
volatile uint8_t  dds1_wave = 0;

volatile uint32_t dds2_phase_acc = 0;
volatile uint32_t dds2_ftw = 0;
volatile uint16_t dds2_vpp = 1000;
volatile uint8_t  dds2_wave = 0;

#define LUT_SIZE 1024
float SineLUT[LUT_SIZE];
float TriLUT[LUT_SIZE];

uint16_t ActiveLUT1[LUT_SIZE];
uint16_t ActiveLUT2[LUT_SIZE];

static uint16_t DDS_Clamp12(float value)
{
    if (value < 0.0f) return 0u;
    if (value > 4095.0f) return 4095u;
    return (uint16_t)value;
}

static void DDS_RebuildActiveLUT(uint16_t *activeLut, uint16_t vpp, uint8_t waveType)
{
    float scale = (float)vpp / 1000.0f;
    for (int i = 0; i < LUT_SIZE; i++) {
        float y = 0.0f;
        if (waveType == 1) { /* SIG_SINE */
            y = SineLUT[i];
        } else if (waveType == 2) { /* SIG_TRIANGLE */
            y = TriLUT[i];
        }
        activeLut[i] = DDS_Clamp12(DAC_BIAS_1_3V + DAC_AMP_0_5V * scale * y);
    }
}

void DDS_Init(void)
{
    for (int i = 0; i < LUT_SIZE; i++) {
        float p = (float)i / (float)LUT_SIZE * 2.0f * DDS_PI;
        SineLUT[i] = sinf(p);
        TriLUT[i] = (2.0f / DDS_PI) * asinf(sinf(p));
    }

    DDS_RebuildActiveLUT(ActiveLUT1, 1000, 1);
    DDS_RebuildActiveLUT(ActiveLUT2, 1000, 1);

    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
    HAL_TIM_Base_Start(&htim6);
}

// =================== DDS1 ===================
void DDS1_Update_DATA(uint32_t freq, uint16_t vpp, uint8_t waveType)
{
    dds1_ftw = (uint32_t)(((uint64_t)freq * 4294967296ULL) / 1000000ULL);
    
    // 如果振幅或波形发生改变，重新生成整数查找表
    if (dds1_vpp != vpp || dds1_wave != waveType) {
        dds1_vpp = vpp;
        dds1_wave = waveType;
        DDS_RebuildActiveLUT(ActiveLUT1, vpp, waveType);
    }
}

void DDS1_Update_FTW(uint32_t ftw)
{
    dds1_ftw = ftw;
}

void DDS1_Add_Phase(int32_t phase_offset)
{
    dds1_phase_acc += phase_offset;
}

void DDS1_Start(void)
{
    // 启动前先填充一下初始数据
    for (int i = 0; i < DDS_BUF_SIZE; i++) {
        dds1_phase_acc += dds1_ftw;
        Buffer1[i] = ActiveLUT1[dds1_phase_acc >> 22];
    }
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)Buffer1, DDS_BUF_SIZE, DAC_ALIGN_12B_R);
}

// =================== DDS2 ===================
void DDS2_Update_DATA(uint32_t freq, uint16_t vpp, uint8_t waveType)
{
    dds2_ftw = (uint32_t)(((uint64_t)freq * 4294967296ULL) / 1000000ULL);
    
    if (dds2_vpp != vpp || dds2_wave != waveType) {
        dds2_vpp = vpp;
        dds2_wave = waveType;
        DDS_RebuildActiveLUT(ActiveLUT2, vpp, waveType);
    }
}

void DDS2_Update_FTW(uint32_t ftw)
{
    dds2_ftw = ftw;
}

void DDS2_Start(void)
{
    for (int i = 0; i < DDS_BUF_SIZE; i++) {
        dds2_phase_acc += dds2_ftw;
        Buffer2[i] = ActiveLUT2[dds2_phase_acc >> 22];
    }
    HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*)Buffer2, DDS_BUF_SIZE, DAC_ALIGN_12B_R);
}

void DDS_Stop(void)
{
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_2);
}

// =================== DMA 回调 ===================
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    for (int i = 0; i < DDS_BUF_SIZE / 2; i++) {
        dds1_phase_acc += dds1_ftw;
        Buffer1[i] = ActiveLUT1[dds1_phase_acc >> 22];
    }
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
    for (int i = DDS_BUF_SIZE / 2; i < DDS_BUF_SIZE; i++) {
        dds1_phase_acc += dds1_ftw;
        Buffer1[i] = ActiveLUT1[dds1_phase_acc >> 22];
    }
}

void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    for (int i = 0; i < DDS_BUF_SIZE / 2; i++) {
        dds2_phase_acc += dds2_ftw;
        Buffer2[i] = ActiveLUT2[dds2_phase_acc >> 22];
    }
}

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef *hdac)
{
    for (int i = DDS_BUF_SIZE / 2; i < DDS_BUF_SIZE; i++) {
        dds2_phase_acc += dds2_ftw;
        Buffer2[i] = ActiveLUT2[dds2_phase_acc >> 22];
    }
}
