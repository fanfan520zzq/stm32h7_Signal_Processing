# OpenOCD / GDB 命令手册

本文件记录当前已确认的 OpenOCD、ST-LINK 和 GDB Server 参数。AI 不得猜测尚未确认的工具路径或串口参数。

## 待确认硬件信息

```text
调试器：ST-LINK V2
OpenOCD 版本：0.12.0 (2023-01-14)
OpenOCD 可执行文件：D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe
OpenOCD scripts：D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts
interface/board cfg：D:\openOCD\DevEnv\stm32h7_stlink.cfg
传输层：hla_swd
适配器速度：1800 kHz
目标：STM32H74x/75x，Cortex-M7
目标电压：约 3.5 V（由 OpenOCD 实测）
GDB Server：3333
Telnet Server：4444（Debug 模式）
复位方式：当前命令使用 `reset init`
GDB 可执行文件：D:\openOCD\DevEnv\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe
```

## 已确认的烧录命令

```powershell
& "D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe" `
  -s "D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts" `
  -f "D:\openOCD\DevEnv\stm32h7_stlink.cfg" `
  -c "tcl_port disabled" `
  -c "gdb_port disabled" `
  -c "program \"C:/Users/Lenovo/Desktop/model/stm32h7_Signal_Processing/cmake-build-debug/IIT6_Oscilliscope.elf\"" `
  -c "reset" `
  -c "shutdown"
```

已确认结果：

- ST-LINK V2 已识别；
- HLA-SWD 已自动选择；
- STM32H74x/75x 已识别；
- 2 MB Flash 已识别为双 Bank；
- Programming Finished；
- reset 和 shutdown 已执行。

## 已确认的 Debug Server 命令

```powershell
& "D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe" `
  -c "tcl_port disabled" `
  -c "gdb_port 3333" `
  -c "telnet_port 4444" `
  -s "D:\openOCD\DevEnv\openocd-v0.12.0-i686-w64-mingw32\share\openocd\scripts" `
  -f "D:\openOCD\DevEnv\stm32h7_stlink.cfg" `
  -c "program C:/Users/Lenovo/Desktop/model/stm32h7_Signal_Processing/cmake-build-debug/IIT6_Oscilliscope.elf" `
  -c "init;reset init;" `
  -c "echo (((READY)))"
```

已确认结果：

- GDB Server 监听 `localhost:3333`；
- Telnet Server 监听 `localhost:4444`；
- ELF 已成功烧录；
- 目标复位后保持 halted，等待 GDB 连接；
- OpenOCD 输出 `(((READY)))`；
- GDB 已成功连接。

## GDB 连接模板

```text
arm-none-eabi-gdb <absolute-path-to-elf>
target extended-remote localhost:3333
monitor reset halt
load
continue
```

已确认 GDB 路径：

```text
D:\openOCD\DevEnv\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe
```

## 路径一致性待确认

当前工程曾存在两种构建目录约定：

```text
cmake-build-debug/IIT6_Oscilliscope.elf
build/Debug/<实际目标文件>
```

当前统一约定：使用 CLion 生成的 `cmake-build-debug`。

已确认当前有效 ELF：

```text
C:\Users\Lenovo\Desktop\model\stm32h7_Signal_Processing\cmake-build-debug\IIT6_Oscilliscope.elf
```

当前 `build/Debug` 没有可用 ELF。烧录和调试必须统一使用同一次构建生成的 `cmake-build-debug` ELF，不能混用两个目录。

后续如果改为完全使用 CMake Preset，再单独完成一次目录迁移和验证，不在本准备阶段混用两套产物。

## 命令模板

以下参数已经明确，但仍需根据 CLion 的实际构建输出确认 ELF 路径：

```powershell
openocd -f "D:\openOCD\DevEnv\stm32h7_stlink.cfg" -c "program C:/Users/Lenovo/Desktop/model/stm32h7_Signal_Processing/cmake-build-debug/IIT6_Oscilliscope.elf verify reset exit"
& "D:\openOCD\DevEnv\GNU-tools-for-STM32\bin\arm-none-eabi-gdb.exe" "C:/Users/Lenovo/Desktop/model/stm32h7_Signal_Processing/cmake-build-debug/IIT6_Oscilliscope.elf"
```

## 诊断信息

首次成功连接后记录：

- OpenOCD 完整启动输出；
- 识别到的调试器；
- 目标芯片 ID；
- 复位后的 PC 和 SP；
- 烧录校验结果；
- 已知失败现象和恢复方法。

## STM32CubeProgrammer

本地也有 STM32CubeProgrammer，可以作为备用烧录工具。暂不把它混入主流程，除非需要：

- OpenOCD 无法连接；
- 特定 H7 双 Bank 操作需要官方工具；
- 需要批量擦除、选项字节或更强的烧录诊断。

使用前仍需记录 `STM32_Programmer_CLI.exe` 的实际路径和连接参数。
