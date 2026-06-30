#include "calib.h"
#include "config.h"
#include "sweep_engine.h"
#include <math.h>
#include <stdio.h>

// 直通校准表: 同源测得的 H_thru(f) = 幅度比 + 相位偏斜.
#define THRU_MAX 48
typedef struct { float f; float mag; float phase; } ThruPt;
static ThruPt g_thru[THRU_MAX];
static int    g_thru_len = 0;

void cal_clear(void) { g_thru_len = 0; }

int cal_is_valid(void) { return g_thru_len >= 2; }

// 跑直通校准: 两路接同一信号源, 对数扫一遍, 记录每点的 H_thru.
// 频率范围 100Hz..FS_MAX, ~10点/十倍程.
void cal_run_thru(void)
{
    g_thru_len = 0;
    float r = powf(10.0f, 1.0f / 10.0f);   // 10 点/十倍程
    float f = 100.0f;
    while (f <= FS_MAX_HZ && g_thru_len < THRU_MAX) {
        float fa, mag, phase;
        sweep_measure_raw(f, &fa, &mag, &phase);
        g_thru[g_thru_len].f     = fa;
        g_thru[g_thru_len].mag   = mag;
        g_thru[g_thru_len].phase = phase;
        g_thru_len++;
        f *= r;
    }
}

// 线性插值取 f 处的 thru 值 (相位偏斜随频率近似线性, 线性插值足够准).
static void thru_lookup(float f, float *mag, float *phase)
{
    if (g_thru_len == 0) { *mag = 1.0f; *phase = 0.0f; return; }
    if (f <= g_thru[0].f)              { *mag = g_thru[0].mag;            *phase = g_thru[0].phase;            return; }
    if (f >= g_thru[g_thru_len-1].f)   { *mag = g_thru[g_thru_len-1].mag; *phase = g_thru[g_thru_len-1].phase; return; }
    for (int i = 1; i < g_thru_len; i++) {
        if (f <= g_thru[i].f) {
            float t = (f - g_thru[i-1].f) / (g_thru[i].f - g_thru[i-1].f);
            *mag   = g_thru[i-1].mag   + t * (g_thru[i].mag   - g_thru[i-1].mag);
            *phase = g_thru[i-1].phase + t * (g_thru[i].phase - g_thru[i-1].phase);
            return;
        }
    }
    *mag = g_thru[g_thru_len-1].mag; *phase = g_thru[g_thru_len-1].phase;
}

// H_corrected = H_meas / H_thru: 除掉两路增益失配, 减掉通道偏斜相位.
// 【硬编码免校准模式】：直接用实测的固有偏斜, 不查表.
// thru 相位 = -2π·f·τ (与实测同号, 负). 校准 = 减去它.
// CAL_SKEW_S 调法: 跑 STAGE3 同源, 让"校准后 phase"在高频(1MHz)≈0 即可.
//   实测 -0.67°@1MHz -> τ = 0.67/360/1e6 ≈ 1.86ns.
//   注意: 该偏斜不同上电有 ~0.3° 漂移, 写死会留 ~0.2-0.3° 残差; 要更准用运行时 thru-cal.
#define CAL_SKEW_S   1.86e-9f
void cal_apply_correction(float f, float* mag, float* phase)
{
    float thru_phase = -2.0f * 3.14159265f * f * CAL_SKEW_S;
    *phase -= thru_phase;       // 减掉偏斜 -> 把同源相位拉回 ~0

    *mag   /= 1.0008f;          // 两路增益失配 ~0.08%, 顺手修掉
}

void cal_print_table(void)
{
    printf("=== THRU CAL (%d pts) ===\r\n", g_thru_len);
    printf("f,mag,phase_deg\r\n");
    for (int i = 0; i < g_thru_len; i++) {
        printf("%.1f,%.5f,%.3f\r\n",
               g_thru[i].f, g_thru[i].mag, g_thru[i].phase * 57.29578f);
    }
}

// 专门为外部手动 DDS 提供的简易校准
void cal_run_thru_manual(const float* freqs, int count)
{
    g_thru_len = 0;
    extern UART_HandleTypeDef huart1;
    
    for(int i=0; i<count; i++) {
        if(g_thru_len >= THRU_MAX) break;
        printf("\r\n>> [直通校准] 请将外部仪器频率设置为 %.0f Hz\r\n", freqs[i]);
        printf(">> 设置完成后，请在串口助手发送小写字母 'y' 继续...\r\n");
        
        uint8_t rx = 0;
        while (rx != 'y' && rx != 'Y') {
            HAL_UART_Receive(&huart1, &rx, 1, HAL_MAX_DELAY);
        }
        
        float fa, mag, phase;
        sweep_measure_raw(freqs[i], &fa, &mag, &phase);
        g_thru[g_thru_len].f     = fa;
        g_thru[g_thru_len].mag   = mag;
        g_thru[g_thru_len].phase = phase;
        g_thru_len++;
    }
}
