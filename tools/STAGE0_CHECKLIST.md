# 阶段验收记录: Stage 0 (重构准备与基建)

## 基本信息

```text
Stage：Stage 0
Profile：Bare-metal & Pin Mapping
Git commit：TODO
Git tag：TODO
固件 ELF：build/Debug/IIT6_Oscilliscope.elf
板卡：STM32H743
日期：2026-07-14
```

## 编译

- [x] CMake 配置成功
- [x] Debug/目标配置编译成功
- [x] 无未解释的新警告
- [x] ELF 生成成功

## 烧录与启动

- [x] OpenOCD 连接成功
- [x] 芯片识别正确
- [x] 烧录校验成功
- [x] 复位后正常启动
- [x] 串口启动信息正确

## 功能验证

```text
输入条件：代码重构文档和引脚映射清理，不影响现有逻辑
仪器设置：无
采样率/输出频率：无
通道和引脚：全外设配置已登记 (见 PIN_MAPPING.md)
期望结果：编译通过且 UART 自动验证通过
实际结果：编译通过，UART PING 自动验证 PASS
```

## 证据和限制

```text
官方资料依据：STM32CubeMX 生成代码
官方例程路径：无
当前板卡证据：自动构建并运行串口回环成功
已确认结论：当前无 FreeRTOS 残留，引脚丝印反置冲突已记录
尚未确认：外设驱动重构（留至 Stage 1/2）
不能外推的范围：ADC 和 DAC 的实际模拟量正确性仍需实机验证
```

## Stage 完成动作

- [x] 更新模块 Markdown (更新 AGENTS.md, REFACTOR_PLAN.md)
- [x] 更新 README/验证记录 (新建 PIN_MAPPING.md 和 STAGE0_CHECKLIST.md)
- [ ] 创建 Git commit
- [ ] 创建 Git tag
- [ ] 整理稳定内容到 `D:\ZYNQ32_MEMRAX`
- [x] 标注 A/B/C/D 证据等级 (B级)
