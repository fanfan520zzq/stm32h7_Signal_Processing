# 复盘：一个 `float` 参数 + 缺失原型 导致 AD9833「电平跳变、无波形」

> 场景：STM32H743 (Cortex-M7, 硬浮点)。扫频模块通过 `dds_set_frequency(float)` 驱动 AD9833。
> 现象：示波器上电平乱跳、没有正弦波；但直接调 `AD9833_SetFixedOutput(10000, WAVE_SINE)` 却完全正常。
> 真凶：**调用处看不到函数原型**，编译器按"隐式声明"把 `float` 当 `double` 传，硬浮点 ABI 下寄存器/位模式对不上，被调函数收到一个**垃圾频率值**。

---

## 1. 现象与排查弯路

一开始症状是"输出异常 / 闪一下就没了"，我们依次怀疑并排除了：

| 怀疑对象 | 结论 |
|---------|------|
| SPI 时钟太快（480MHz 后 SCLK 超 AD9833 上限） | 否。PLL1Q=192MHz，/64=3MHz，远低于 40MHz 上限 |
| AD9833 复位位没清 / 初始化时序 | 否。驱动和已验证分支逐字一致 |
| 切频不重写控制字导致掉输出 | 否。同上 |
| SPI / GPIO / 时钟树配置变了 | 否。与已验证分支 diff 全为空或无关 |

**关键对比**让真凶现形：

- `main.c` 里**直接**调 `AD9833_SetFixedOutput(10000, WAVE_SINE)` → 正常输出 ✅
- `main.c` 里调 `dds_set_frequency(10000.0f)`（它内部也只是转调 `AD9833_SetFixedOutput`）→ 坏波形 ❌

两条路最后都进同一个函数，唯一差别是**中间隔了一层 `float` 参数的 `dds_set_frequency`**。编译时其实早有提示：

```
main.c:124: warning: implicit declaration of function 'dds_set_frequency'
            [-Wimplicit-function-declaration]
```

这条 warning 一直被忽略——**这正是教训本身**。

---

## 2. 根因：隐式声明 + 默认实参提升 + 硬浮点 ABI

### 2.1 函数定义和调用分离

- 定义在 `hooks.c`：`void dds_set_frequency(float hz) { ... }`
- 调用在 `main.c`，但 `main.c` **没有 include 任何声明它的头文件**。
- `sweep_engine.c` 里有 `extern void dds_set_frequency(float hz);`，所以**它调用时是对的**——只有 `main.c` 漏了原型。

### 2.2 没有原型时，C 怎么处理调用？

C 语言里，调用一个**没有可见声明**的函数时（C99 起这本是错误，但编译器通常降级为 warning 继续编），对实参应用 **默认实参提升（default argument promotions）**：

- `float` → **提升为 `double`**
- `char` / `short` → 提升为 `int`

所以 `dds_set_frequency(10000.0f)` 在调用侧被当成"传一个 `double`"。

### 2.3 硬浮点 ABI：`float` 和 `double` 走不同寄存器

本工程编译选项是 `-mfloat-abi=hard`（硬浮点）。按 ARM AAPCS：

| 类型 | 传参位置 | 位宽 |
|------|---------|------|
| `float`  | `s0`（单精度寄存器） | 32-bit |
| `double` | `d0`（双精度寄存器，物理上覆盖 s0/s1） | 64-bit |

于是出现致命错位：

```
调用侧（无原型）：把 10000.0 当 double 写进 d0
被调侧（真实签名 float）：从 s0 读 32-bit 当作 float
```

`d0` 里的 double `10000.0` 低 32 位 ≠ float `10000.0` 的位模式。被调函数读到的 `hz` 是个**毫无意义的数**——可能是几兆、零、或 NaN。

> 直观理解：
> - `float 10000.0f` 的 32-bit 位模式 = `0x461C4000`
> - `double 10000.0` 的 64-bit 位模式 = `0x40C3880000000000`
> - 从 s0（= d0 的低 32 位）读出来的是 `0x00000000` → `hz = 0.0`，甚至更糟。

### 2.4 垃圾频率 → AD9833 输出乱

`dds_set_frequency` 把这个垃圾值算成 AD9833 频率寄存器，写进芯片 → DAC 输出一个错误/越界频率 → 示波器上就是"电平跳变、无波形"。

而 `sweep_engine.c`（有 `extern` 原型）和 `main.c` 直接调 `AD9833_SetFixedOutput`（有头文件原型、且参数是 `uint32_t` 不涉及 float 提升）这两条路都正确，所以它们正常。

---

## 3. 修复

在所有调用方都能看到的共享头文件（`config.h`）里补上原型：

```c
// 生成端 hook (实现在 hooks.c)
void dds_set_frequency(float hz);
```

有了原型，编译器知道参数是 `float`，调用侧就把它写进 `s0`，与被调侧一致。warning 消失，输出立刻正常。

---

## 4. 教训清单（可直接套用）

1. **绝不忽略 `-Wimplicit-function-declaration` 警告。**
   它不是"风格问题"，对 `float`/`double`/结构体参数会**静默传错值**，且现象诡异、极难定位。

2. **跨文件函数一律在头文件里声明。** 函数定义在 `a.c`、被 `b.c` 调用，就必须有个 `a.h`（或共享头）声明它，且双方都 include。别靠"恰好能链接上"。

3. **硬浮点目标尤其危险。** `-mfloat-abi=hard` 下 `float`↔`double` 的寄存器差异会把缺原型的 bug 放大成数据损坏；软浮点可能"碰巧"对齐而掩盖问题。

4. **建议把这类 warning 升级为 error**，从源头堵死：
   ```cmake
   target_compile_options(${TARGET} PRIVATE
       -Werror=implicit-function-declaration
       -Werror=implicit-int)
   ```

5. **排查诡异硬件现象时，先看编译 warning。** 这次绕了一大圈查 SPI/时钟/驱动时序，而答案早就印在 build log 里。

---

## 5. 一句话总结

> 缺函数原型 + `float` 实参 + ARM 硬浮点 ABI = 频率值在寄存器里被悄悄"调包"。
> 一条被忽略的 `implicit declaration` 警告，足以让你怀疑整条 SPI 链路。
