# AD9833 STM32 HAL库驱动设计指南

本指南用于指导使用STM32 HAL库重构AD9833波形发生器模块的底层与应用代码，实现定频、扫频与幅值可变的需求。

## 1. 硬件通信接口 (SPI/GPIO模拟)

AD9833 使用单向的 SPI 协议（DAT, CLK, FNC）。C51例程使用的是软件模拟SPI，建议在STM32 HAL库中封装底层写16位数据的接口。可以选择 **硬件SPI** 或 **软件模拟SPI**。

```c
// 软件模拟SPI 伪代码参考
void AD9833_Write16(uint16_t data) {
    HAL_GPIO_WritePin(FSYNC_PORT, FSYNC_PIN, GPIO_PIN_RESET);
    for(int i=0; i<16; i++) {
        if(data & 0x8000) 
            HAL_GPIO_WritePin(DAT_PORT, DAT_PIN, GPIO_PIN_SET);
        else 
            HAL_GPIO_WritePin(DAT_PORT, DAT_PIN, GPIO_PIN_RESET);
        
        HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(CLK_PORT, CLK_PIN, GPIO_PIN_SET);
        data <<= 1;
    }
    HAL_GPIO_WritePin(FSYNC_PORT, FSYNC_PIN, GPIO_PIN_SET);
}
```

## 2. 核心功能需求实现

### 2.1 定频输出 (Fixed Frequency)
必须包含频率设定函数。AD9833频率寄存器是28位的，写入分两步（连续两次写入或一次完整写入），计算公式： 
> $ Freq\_Reg = \frac{F_{out} \times 2^{28}}{F_{MCLK}} $

通常 $F_{MCLK}$ = 25MHz。寄存器需拆分为两个14位的数据分别写入。

### 2.2 波形切换 (Waveform Selection)
根据原始C51例程，可以通过配置控制寄存器选定不同波形：
- 正弦波 (Sine): `0x2000`
- 三角波 (Triangle): `0x2002`
- 方波 (Square): `0x2020` 或 `0x2028`

### 2.3 扫频输出 (Frequency Sweeping)
建议创建一个非阻塞或定时器驱动的扫频函数。
- **机制**：从 `Freq_Start` 到 `Freq_End`，按照 `Step`（步进）和 `TimeRange`（每步时间间隔）进行更新。
- **技巧**：AD9833具备 `FREQ0` 和 `FREQ1` 两个频率寄存器，可以通过后台预先写入下一个频率的寄存器，然后切换引脚或控制位实现无缝切频，从而避免频率突变停顿。

### 2.4 幅值可变输出 (Variable Amplitude) 注意事项与对策
**极其重要的硬件限制：** AD9833 芯片自身 **不支持** 通过内部寄存器直接调节输出电压波形的相对幅值（其DAC满量程由外部电阻决定，输出通常固定在 0.6V p-p 左右）。
- **如果你的模块带有数字电位器（如MCP41xxx）或AGC电路**：你需要编写相应的程控增益驱动（SPI/I2C控制电位器）。
- **如果你的模块没有外接数字芯片**：只能依靠外部手动旋钮电位器，软件层**无法**凭空实现“幅值可变输出”。在代码生成时应明确此模块的具体外围链路，或给出利用外接 DAC / 数字电位器调节电压参考的接口预留。

## 3. 面向AICoding的具体开发步骤

1. **提供引脚与系统时钟配置**：确认 MCLK 大小（多数模块采用 25MHz 有源晶振）。
2. **编写底层寄存器写入宏或内联函数**：实现 `Write16()`。
3. **架构头文件 (`ad9833.h`)**：
   - 枚举波形类型：`typedef enum { AD9833_SINE, AD9833_SQUARE, AD9833_TRIANGLE } Waveform_t;`
   - 定义参数结构体。
4. **架构源文件 (`ad9833.c`)**：
   - 编写 `AD9833_Init()`
   - 编写频率设置 `AD9833_SetFrequency(uint8_t RegNum, uint32_t freq, Waveform_t type)`
   - 编写扫频轮询/定时器服务程序 `AD9833_SweepTick()`
5. **整合资源**：将代码输出到目标工程中进行引脚绑定。