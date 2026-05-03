#include "LCD.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>



#define TX_BUF_SIZE  512
static uint8_t tx_buf_a[TX_BUF_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
static uint8_t tx_buf_b[TX_BUF_SIZE] __attribute__((section(".dma_buffer"))) __attribute__((aligned(32)));
static uint8_t *tx_front = tx_buf_a;   // 正在发送
static uint8_t *tx_back  = tx_buf_b;   // 正在填充

static inline void lcd_wait_dma(void) {
    uint32_t timeout = HAL_GetTick() + 100;
    while ((huart1.gState & HAL_UART_STATE_BUSY_TX)
           && HAL_GetTick() < timeout)
    {
        __NOP();
    }
}


void lcd_send_raw(const uint8_t *data, uint16_t len) {
    lcd_wait_dma();
    memcpy(tx_front, data, len);
    // SCB_CleanDCache_by_Addr((uint32_t*)tx_front, len);
    HAL_UART_Transmit_DMA(&huart1, tx_front, len);
    // 交换缓冲（下次填充另一块，不影响本次 DMA）
    uint8_t *tmp = tx_front;
    tx_front = tx_back;
    tx_back = tmp;
}


void lcd_cmd(const char *fmt, ...) {
    static uint8_t cmd_buf[128];  // 不需要 dma_buffer，因为会被 memcpy 进 tx_front
    va_list args;
    va_start(args, fmt);
    int n = vsnprintf((char*)cmd_buf, sizeof(cmd_buf) - 3, fmt, args);
    va_end(args);
    cmd_buf[n]   = 0xFF;
    cmd_buf[n+1] = 0xFF;
    cmd_buf[n+2] = 0xFF;
    lcd_send_raw(cmd_buf, n + 3);
}


void LCD_Update_Stats(float ch1_freq, float ch1_vpp, uint8_t ch1_type,
                      float ch2_freq, float ch2_vpp, uint8_t ch2_type) {
    // 频率用 0.1Hz 精度存成整数，避免浮点格式化
    lcd_cmd("va0.val=%d", (int)(ch1_freq * 10));
    lcd_cmd("va1.val=%d", (int)(ch1_vpp ));
    lcd_cmd("va2.val=%d", ch1_type);
    lcd_cmd("va3.val=%d", (int)(ch2_freq * 10));
    lcd_cmd("va4.val=%d", (int)(ch2_vpp ));
    lcd_cmd("va5.val=%d", ch2_type);
}


static const uint8_t SinTable[PERIOD] = {
    128,138,148,158,167,177,186,194,203,211,218,225,232,237,242,246,249,252,254,255,
    255,254,252,249,246,242,237,232,225,218,211,203,194,186,177,167,158,148,138,128,
    118,108,98,89,79,70,62,53,45,38,31,24,19,14,10,7,4,2,1,0,
    0,1,2,4,7,10,14,19,24,31,38,45,53,62,70,79,89,98,108,118
};

void LCD_Update_Waves(uint8_t Type, uint16_t Amplitude, uint8_t CH, float freq) {
    uint8_t wave_buffer[LCD_WAVE_POINTS];

    // 计算偏移量缩放：0-65535 映射到 0-160
    uint32_t scale_160 = (uint32_t)(Amplitude * 160) >> 16;
    uint8_t top_limit = 32; // 顶值基准

    for (int i = 0; i < LCD_WAVE_POINTS; i++) {
        uint8_t phase = i % PERIOD;
        uint32_t temp_offset = 0;

        if (Type==SINE)  temp_offset = (SinTable[phase] * scale_160) >> 8;
        else if (Type==SQUARE) {                   // Square
            temp_offset = (phase < PERIOD/2) ? 0 : scale_160;
        }
        else if (Type==TRIANGLE) {                   // Triangle
            uint32_t half = PERIOD / 2;
            uint32_t tri;
            if (phase < half)
                tri = phase;                    // 0 → half-1，上升
            else
                tri = PERIOD - 1 - phase;      // half → 0，下降
            temp_offset = (tri * scale_160) / (half - 1);
        }
        else if (Type==DC) {                   // dc
            wave_buffer[i] = LCD_WAVE_HEIGHT / 2;
            continue;
        }

        // 核心修改：从底值 32 开始向上加
        // 结果范围：32 + (0~160) = 32 ~ 192
        wave_buffer[i] = top_limit + (uint8_t)temp_offset;
    }

    // 发送波形数据
    lcd_cmd("addt %d,%d,%d", LCD_COMP_WAVE, CH, LCD_WAVE_POINTS);
    HAL_Delay(10);
    // 3. 发原始字节（    无 \xFF 分隔符） 没问题
    lcd_send_raw(wave_buffer, LCD_WAVE_POINTS);
    int a=1;
}