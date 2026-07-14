# AI 辅助开发约束与验证流程

## 1. 目标

本工程允许 AI 协助查阅资料、设计、编码、编译、调试、烧录和验证，但所有硬件结论必须有可追溯证据。

AI 的工作原则：

- 积极查阅本地官方资料和已验证工程；
- 优先复用官方例程中的外设配置思路；
- 不把 AI 推测写成芯片事实；
- 不把一次板卡演示结果写成完整性能指标；
- 每个阶段以可编译、可运行、可验证为完成条件；
- 发现官方资料与当前工程冲突时，保留冲突记录并暂停扩大修改范围。

---

## 2. 资料来源优先级

### A 级：官方原始资料

可直接作为高参考性依据，但必须记录具体位置：

- STM32H743 官方参考手册；
- STM32H743 官方数据手册；
- STM32H743 官方勘误手册；
- ST 官方 STM32CubeH7 包中的 `Templates`、`Templates_LL`、`Examples`、`Examples_LL`；
- ST 官方 HAL/LL 驱动头文件、源文件和 API 注释。

A 级资料笔记必须标注：

```text
来源文件：STM32H743参考手册.pdf
章节：xx
小节：xx
页码：xx
对应工程路径：STM32Cube_FW_H7_V1.12.1/Projects/...
对应符号：LL_xxx / HAL_xxx / 寄存器字段 xxx
适用范围：STM32H743xx / 当前板卡 / 特定外设实例
```

### B 级：本板卡已经验证的模板

- 已在当前板卡下载运行；
- 有测试条件、仪器或串口输出记录；
- 有明确的输入、输出和限制；
- 可以作为后续模块的复用模板。

### C 级：审阅过但尚未板卡验证的资料

- 经过人工或 AI 复核；
- 来源可靠；
- 但尚未在当前芯片、当前板卡或当前接线下完整验证。

### D 级：待审核内容

- AI 推理；
- 网络文章；
- 不完整的实验记录；
- 没有明确出处的经验代码。

D 级内容不得直接作为硬件配置结论。

---

## 3. 查阅官方资料时的强制动作

当开发过程中查阅 PDF、官方例程或驱动源码，并且该内容影响代码设计时，AI 应优先生成一个模块说明 Markdown，而不是只把结论写进代码注释。

模块说明至少包含：

1. 模块目的；
2. 当前工程使用的外设实例；
3. 参考资料的文件名、章节、小节和页码；
4. 官方例程的相对路径；
5. 参考的关键 API、宏、寄存器字段或初始化顺序；
6. 当前工程与官方例程的差异；
7. 已验证内容；
8. 尚未验证内容；
9. 适用芯片和硬件边界；
10. 资料等级，官方直接依据默认为 A 级。

推荐命名：

```text
docs/modules/<module_name>.md
```

示例：

```text
docs/modules/adc_external_trigger_ll.md
docs/modules/dac_waveform_trigger_ll.md
docs/modules/dma_buffer_h7.md
docs/modules/si5351_timer_external_clock.md
```

### 官方资料引用模板

```markdown
# 模块名称

## 资料等级

A：直接依据 ST 官方资料。

## 官方来源

- 文件：STM32H743参考手册.pdf
- 章节：
- 小节：
- 页码：
- 工程：STM32Cube_FW_H7_V1.12.1/Projects/...
- 示例名称：

## 当前工程采用方式

...

## 与官方例程的差异

...

## 已验证

- [ ] 编译通过
- [ ] 下载成功
- [ ] 外设启动
- [ ] 波形/数据结果正确

## 尚未验证与限制

...
```

---

## 4. 官方 STM32Cube 资料使用约定

当前本地官方包：

```text
C:\Users\Lenovo\STM32Cube\Repository\STM32Cube_FW_H7_V1.12.1
```

当前已确认的重点目录：

```text
Projects/Templates_LL
Projects/Examples_LL
Projects/NUCLEO-H743ZI/Examples_LL
Projects/STM32H743I-EVAL/Examples_LL
```

与本模板优先相关的官方例程方向：

- ADC 外部触发和 DMA；
- DAC 硬件触发波形输出；
- TIM 触发、TRGO 和外部时钟；
- DMA 配置和传输完成处理；
- UART、SPI、I2C 的 LL 示例；
- H7 Cache、RAM 区域和 DMA 相关模板。

官方例程可以参考配置和调用顺序，但不能直接假定其引脚、时钟、DMA Stream、Request 或板级连接适用于当前工程。

---

## 5. 编译、Debug 和烧录的 AI 工作流

### 5.1 推荐工具链

当前工具链：

```text
CLion + CMake/Ninja + arm-none-eabi-gcc + OpenOCD
```

这个组合适合 AI 辅助开发，原因是：

- CMake 命令可重复；
- 编译命令容易自动化；
- OpenOCD 支持脚本化烧录和 GDB 调试；
- 终端输出适合 AI 阅读和分析；
- 不依赖复杂 IDE 图形操作。

### 5.2 AI 不应猜测的内容

AI 不得自行猜测以下配置：

- OpenOCD interface 文件；
- target 文件；
- 调试器型号；
- SWD/JTAG 接口速度；
- 芯片复位方式；
- ELF 文件路径；
- 串口号和波特率；
- 实际板卡连接。

这些内容应写入工程命令手册或配置文件。

### 5.3 建议建立的命令手册

建议建立：

```text
tools/BUILD_DEBUG.md
tools/OPENOCD_COMMANDS.md
tools/VERIFY_CHECKLIST.md
```

至少记录：

```text
配置命令
Debug 编译命令
Release 编译命令
清理构建命令
ELF/BIN/HEX 输出位置
OpenOCD 烧录命令
OpenOCD 复位命令
GDB 连接命令
串口监视命令
硬件验收步骤
```

命令手册的目的不是限制 AI，而是让 AI 使用真实、可复现的工程命令，避免凭经验猜测工具链参数。

---

## 6. Stage 验收、版本管理和知识库入库

每次 Stage 验收完成后必须执行以下流程：

1. 保存源码和文档；
2. 编译验证；
3. 下载或烧录验证；
4. 按该 Stage 的验收表进行板卡测试；
5. 记录测试条件、输出结果和限制；
6. 更新 README 或 Stage 记录；
7. 创建 Git 提交；
8. 创建 Git 标签或版本号；
9. 将稳定结论整理到 `D:\ZYNQ32_MEMRAX`；
10. 在知识库中标注证据等级和源码版本。

推荐版本格式：

```text
stage-00-baseline
stage-01-structure
stage-02-profile
stage-03-clock
stage-04-adc-capture
stage-05-pa4-dds
stage-06-protocol
stage-07-analysis
```

知识库入库时必须区分：

```text
官方资料结论：A 级
当前板卡已验证模板：B 级
已审阅未验证设计：C 级
待审核 AI 笔记：D 级
```

任何“本次实验可以工作”的结论，都不得自动升级为“芯片/模块的完整能力范围”。

---

## 7. AI 每次开发前后的检查清单

### 开发前

- [ ] 明确当前 Stage 和目标；
- [ ] 检查当前 Profile；
- [ ] 查阅相关官方资料或确认无需查阅；
- [ ] 确认 HAL/LL 外设所有权；
- [ ] 确认 CubeMX 生成代码边界；
- [ ] 确认当前板卡引脚和 DMA 配置。

### 开发后

- [ ] 编译通过；
- [ ] 无新增警告或已解释警告；
- [ ] 关键模块状态可查询；
- [ ] 运行验证完成；
- [ ] 官方依据已记录；
- [ ] 未验证内容已标注；
- [ ] Stage 完成后已提交 Git；
- [ ] 稳定结论已整理到知识库。
