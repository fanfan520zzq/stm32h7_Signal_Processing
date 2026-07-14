# AD9833 STM32 HAL 库硬件 SPI 驱动设计指南

## 1. 模块硬件架构（两个芯片共享 SPI 总线）

```
                    ┌─────────────────┐
STM32   SCK ────────┤ AD9833 (DDS)    │
        DAT ────────┤                 │
        FSYNC ──────┤ 片选: FSYNC (Pin15)
                    └────────┬────────┘
                             │ 信号输出
                    ┌────────┴────────┐
        CS  ────────┤ 数字电位器      │ ← 控制幅度
                    │ (MCP41010 等)   │
                    └─────────────────┘
```

| 片选引脚 | 控制对象 | 功能 |
|----------|----------|------|
| FSYNC (GPIO 引脚) | AD9833 本身 | 设频率、波形 |
| CS (GPIO 引脚) | 外部调幅芯片（数字电位器） | 控制输出幅度（0~255） |

两个设备共用 SCK 和 DAT 引脚，各自独立片选，分时操作不冲突。

---

## 2. AD9833 SPI 时序

### 2.1 时序参数

| 参数 | 值 |
|------|-----|
| SPI 模式 | **Mode 3**（CPOL=1, CPHA=1） |
| 时钟空闲电平 | **高电平** |
| 数据采样沿 | **上升沿**（SCK 由低→高时 AD9833 锁存） |
| 数据移位沿 | 下降沿 |
| 数据长度 | **16 位**，MSB 先发 |
| FSYNC | 传输前拉低，**整个 16 位过程保持低**，结束后拉高 |

### 2.2 CubeMX 硬件 SPI 配置

```
SPI Mode:      Transmit Only Master (或 Full-Duplex Master)
Frame Format:  Motorola
Data Size:     8 bits（发两字节） 或 16 bits
CPOL:          HIGH
CPHA:          2 Edge  (即 Mode 3)
NSS:           Software（用普通 GPIO 控制 FSYNC）
Baud Rate:     ≤ 10MHz（AD9833 最高 40MHz，建议用 分频后≤10M）
```

### 2.3 硬件 SPI 写入函数

**8 位模式（推荐，兼容性更好）：**

```c
static void AD9833_Write16(uint16_t data)
{
    uint8_t buf[2];
    buf[0] = (data >> 8) & 0xFF;  // 高字节先发
    buf[1] = data & 0xFF;

    HAL_GPIO_WritePin(FSYNC_PORT, FSYNC_PIN, GPIO_PIN_RESET); // 拉低片选
    HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FSYNC_PORT, FSYNC_PIN, GPIO_PIN_SET);   // 拉高片选
}
```

**16 位模式（SPI 设 Data Size=16bits）：**

```c
static void AD9833_Write16(uint16_t data)
{
    HAL_GPIO_WritePin(FSYNC_PORT, FSYNC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(FSYNC_PORT, FSYNC_PIN, GPIO_PIN_SET);
}
```

> FSYNC 是普通 GPIO，与硬件 SPI 的 NSS 无关，用 `HAL_GPIO_WritePin` 手动控制。

---

## 3. AD9833 控制寄存器详解（16 位）

### 3.1 控制寄存器位（D15:D14 = 00）

```
15 14 13 12 11 10  9  8  7  6  5  4  3  2  1  0
 0  0 B28 HLB FS  PS  RST S1 S12 OPB SP DV2 X  X MO X
```

| 位 | 名称 | 说明 |
|----|------|------|
| D13 | B28 | **1** = 28 位频率分两次写入（先 LSB 后 MSB，必须置 1） |
| D12 | HLB | 当 B28=0 时选择高/低 14 位；B28=1 时忽略 |
| D11 | FSELECT | 频率寄存器选择：0=FREQ0, 1=FREQ1 |
| D10 | PSELECT | 相位寄存器选择 |
| D9 | RESET | **1** = 复位内部寄存器（初始化和扫频切换前用） |
| D8 | SLEEP1 | 1 = 关闭内部 MCLK（省电用） |
| D7 | SLEEP12 | 1 = 关闭内部 DAC（省电用） |
| D6 | OPBITEN | **1** = 启用方波输出 |
| D5 | SIGN/PIB | 方波来源：0=DAC MSB, 1=比较器输出 |
| D4 | DIV2 | 方波分频：0=不分频, 1=÷2 |
| D1 | MODE | 0=正弦/三角波输出, 1=方波使能（与 D6 配合） |

### 3.2 常用控制字

| 控制字 | 二进制（仅标注关键位） | 含义 |
|--------|------------------------|------|
| `0x0100` | `0000 0001 0000 0000` | 置位 RESET（复位） |
| `0x2100` | `00_1_0_0_0_0_1_00000000` | B28=1 + RESET=1，准备写 28 位频率 |
| `0x2000` | B28=1，其余=0 | **正弦波**（退出复位） |
| `0x2002` | B28=1, MODE=1 | **三角波** |
| `0x2020` | B28=1, OPBITEN=1 | **方波**（DAC MSB 输出，不分频） |
| `0x2028` | B28=1, OPBITEN=1, BIT3=1 | 方波（C51 例程值，可兼容） |
| `0x2130` | B28=1, FSELECT=1, RESET=1 | 切换到 FREQ1 并复位 |

### 3.3 频率写入公式

```
Freq_Reg = (Fout × 2^28) / MCLK

2^28 = 268435456
```

典型 MCLK = **25 MHz**（模块上有源晶振）。将计算结果拆为两个 14 位：

```c
uint32_t FreqReg_val = (uint32_t)((freq * 268435456.0) / 25000000.0);
uint16_t LSB_14 = FreqReg_val & 0x3FFF;
uint16_t MSB_14 = (FreqReg_val >> 14) & 0x3FFF;

// FREQ0
AD9833_Write16(LSB_14 | 0x4000);  // D15=0, D14=1 → FREQ0
AD9833_Write16(MSB_14 | 0x4000);

// FREQ1
AD9833_Write16(LSB_14 | 0x8000);  // D15=1, D14=0 → FREQ1
AD9833_Write16(MSB_14 | 0x8000);
```

---

## 4. 幅度控制（外部数字电位器）

AD9833 芯片**本身不支持幅度调节**，模块上额外集成了一个数字电位器（如 MCP41010）。

写入格式（同一 SPI 总线，独立 CS 片选）：

```c
void AD9833_AmpSet(uint8_t amp)   // amp: 0(最小) ~ 255(最大)
{
    HAL_GPIO_WritePin(AMP_CS_PORT, AMP_CS_PIN, GPIO_PIN_RESET); // 选中电位器
    
    uint16_t data = 0x1100 | amp;  // 0x11 = MCP41010 写命令
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&data, 1, HAL_MAX_DELAY);
    
    HAL_GPIO_WritePin(AMP_CS_PORT, AMP_CS_PIN, GPIO_PIN_SET);   // 释放
}
```

> 操作调幅芯片时，FSYNC 必须保持高电平（AD9833 未被选中）。

---

## 5. 初始化流程

```c
void AD9833_Init(void)
{
    AD9833_Write16(0x0100);    // ① 复位
    AD9833_Write16(0x2100);    // ② B28=1（准备写 28 位频率）+ 保持复位
    AD9833_Write16(0x2000);    // ③ 退出复位，输出正弦波
    AD9833_SetFrequency(FREQ_REG_0, 1000);  // ④ 初始频率 1kHz
    AD9833_AmpSet(255);         // ⑤ 初始幅度最大
}
```

---

## 6. 头文件新增内容

在已有定义基础上增加：

```c
/* 外部调幅芯片 CS 引脚（根据原理图修改） */
#define AMP_CS_PORT     GPIOB
#define AMP_CS_PIN      GPIO_PIN_12

/* 函数声明 */
void AD9833_AmpSet(uint8_t amp);  // 新增调幅
```

---

## 7. 与旧版软件模拟 SPI 的对照

| 项目 | 软件模拟 SPI（旧版） | 硬件 SPI（新版） |
|------|---------------------|-----------------|
| SCK 控制 | GPIO 手动翻转 | SPI 外设自动产生 |
| DAT 控制 | GPIO 逐位写 | SPI 外设串行移位 |
| FSYNC 控制 | 手动拉低/拉高 | 仍用 GPIO 手动控制 |
| 写入函数 | `for` 循环 16 次位操作 | `HAL_SPI_Transmit` 一次完成 |
| 调幅 | 无此功能 | `AD9833_AmpSet()` |

---

## 8. 常见问题

1. **输出频率不准** → 检查 `AD9833_MCLK_FREQ` 是否与实际晶振一致（多数为 25MHz）
2. **波形不对** → 检查 Mode 3 配置；示波器看 SCK 空闲是否高电平
3. **调幅无效** → 确认 CS 引脚 GPIO 初始化正确，确认 `0x1100` 命令与模块上实际外设匹配
4. **扫频卡顿** → 可用两个频率寄存器（FREQ0/FREQ1）交替写入，用 FSELECT 位切换，实现无缝切频
