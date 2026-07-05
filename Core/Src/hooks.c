#include "main.h"
#include "ad9833_hal.h"

// 本模块需要的外部接口实现

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
