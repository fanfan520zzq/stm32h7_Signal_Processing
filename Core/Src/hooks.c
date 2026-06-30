#include "main.h"
#include "ad9833_hal.h"

// 本模块需要的外部接口实现

// 生成端: 设置 DDS 输出频率(Hz). 本模块只负责"设频", 不管幅度链与硬件细节.
// 直接走经过验证的 AD9833_SetFixedOutput (= main 226 行调试用的同一条路径).
// 频率取整数 Hz (四舍五入); AD9833 分辨率 ~0.093Hz, 取整误差 <=0.5Hz.
void dds_set_frequency(float hz) {
    if (hz < (float)AD9833_FREQ_MIN) hz = (float)AD9833_FREQ_MIN;
    if (hz > (float)AD9833_FREQ_MAX) hz = (float)AD9833_FREQ_MAX;

    AD9833_SetFixedOutput((uint32_t)(hz + 0.5f), WAVE_SINE);
}

// 生成端: 设置驱动幅度等级(自动量程用). 本范围可空实现.
__weak void gen_set_drive_level(float level) {
    // 默认空实现
}

// 可选: 切前端量程
__weak void afe_set_range(int range_idx) {
    // 默认空实现
}

// 可选: 模式切换的继电器/开关矩阵控制
__weak void switch_set_mode(int mode) {
    // 默认空实现
}
