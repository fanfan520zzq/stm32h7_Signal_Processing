#include "SerialCmdTask.h"
#include "DPLLTask.h"

// 外部依赖：串口读取单字节接口
extern uint8_t UART1_Read_Byte(uint8_t *byte);

// ============================================================
// 串口指令解包与执行模块
// ============================================================
// 通信协议格式: 0xAF [CMD/Data_L] [PARAM/Data_H] 0xFA
// ============================================================
void SerialCmdTask_Poll(void) {
    static uint8_t rx_state = 0;
    static uint8_t cmd_buf[4];
    uint8_t rx_byte;

    // 不断读取环形缓冲区，直到读空
    while (UART1_Read_Byte(&rx_byte)) {
        switch (rx_state) {
            case 0:
                if (rx_byte == 0xAF) {
                    cmd_buf[0] = rx_byte;
                    rx_state = 1;
                }
                break;
            case 1:
                cmd_buf[1] = rx_byte;
                rx_state = 2;
                break;
            case 2:
                cmd_buf[2] = rx_byte;
                rx_state = 3;
                break;
            case 3:
                cmd_buf[3] = rx_byte;
                if (rx_byte == 0xFA) {
                    // 解包成功，提取中间两个字节作为 16位 整数 (小端模式: 低位在前)
                    uint16_t payload = (cmd_buf[2] << 8) | cmd_buf[1];

                    if (payload == 0xFF12) {
                        // AF 12 FF FA: 开始数字移相器自动模式，默认0度
                        dpll_enable = 1;
                        user_phase_shift_deg = 0.0;
                    } 
                    else if (payload == 0xFF23) {
                        // AF 23 FF FA: 待机状态
                        dpll_enable = 0;
                    }
                    else {
                        // 其他值视为直接设置相位 (例如 AF 68 01 FA -> 0x0168 = 360)
                        dpll_enable = 1; // 调节相位时自动开启
                        user_phase_shift_deg = (double)payload;
                        
                        // 安全范围限制
                        while (user_phase_shift_deg >= 360.0) user_phase_shift_deg -= 360.0;
                        while (user_phase_shift_deg < 0.0) user_phase_shift_deg += 360.0;
                    }
                }
                rx_state = 0; // 回到初始状态准备接下一帧
                break;
        }
    }
}
