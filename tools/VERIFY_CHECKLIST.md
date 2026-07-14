# 阶段验收记录模板

## 基本信息

```text
Stage：TODO
Profile：TODO
Git commit：TODO
Git tag：TODO
固件 ELF：TODO
板卡：TODO
日期：TODO
```

## 编译

- [ ] CMake 配置成功
- [ ] Debug/目标配置编译成功
- [ ] 无未解释的新警告
- [ ] ELF 生成成功

## 烧录与启动

- [ ] OpenOCD 连接成功
- [ ] 芯片识别正确
- [ ] 烧录校验成功
- [ ] 复位后正常启动
- [ ] 串口启动信息正确

## 功能验证

```text
输入条件：
仪器设置：
采样率/输出频率：
通道和引脚：
期望结果：
实际结果：
```

## 证据和限制

```text
官方资料依据：
官方例程路径：
当前板卡证据：
已确认结论：
尚未确认：
不能外推的范围：
```

## Stage 完成动作

- [ ] 更新模块 Markdown
- [ ] 更新 README/验证记录
- [ ] 创建 Git commit
- [ ] 创建 Git tag
- [ ] 整理稳定内容到 `D:\ZYNQ32_MEMRAX`
- [ ] 标注 A/B/C/D 证据等级
