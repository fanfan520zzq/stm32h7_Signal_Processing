# Build / Debug 命令手册

## 工具链

当前工程使用：

```text
CMake + Ninja
arm-none-eabi-gcc
CLion
OpenOCD（待填写实际版本和路径）
```

## 检查工具链

```powershell
cmake --version
ninja --version
arm-none-eabi-gcc --version
openocd --version
```

如果命令不在 PATH 中，记录实际绝对路径：

```text
arm-none-eabi-gcc：TODO
openocd：TODO
gdb：TODO
```

## 配置和编译

在工程根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

其他配置：

```powershell
cmake --preset Release
cmake --build --preset Release
```

## 输出文件

当前 AI/烧录/调试统一使用的输出目录：

```text
cmake-build-debug/
```

已确认输出文件：

```text
ELF：C:\Users\Lenovo\Desktop\model\stm32h7_Signal_Processing\cmake-build-debug\IIT6_Oscilliscope.elf
MAP：C:\Users\Lenovo\Desktop\model\stm32h7_Signal_Processing\cmake-build-debug\IIT6_Oscilliscope.map
BIN：TODO
HEX：TODO
```

注意：`CMakePresets.json` 当前定义的是 `build/Debug`，而 CLion 当前有效产物位于 `cmake-build-debug`。在正式改造 CMake Preset 之前，烧录和调试统一使用后者，避免混用 ELF。

## AI 使用规则

- 编译前先确认当前 Profile 和 Stage；
- 不要直接删除构建目录，除非用户明确要求；
- 编译失败时保留完整首个错误和相关上下文；
- 只在确认工具链路径后执行烧录；
- 每次硬件验证记录使用的 ELF 绝对路径和 Git 提交。
