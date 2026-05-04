## stm32接收命令

强迫复位：AF 01 01 FA 
1khz下输入电阻，输出电阻，增益一键测量:AF 12 12 FA
幅频特性测量:AF 23 23 FA
学习当前电路:AF 34 34 FA
测量当前故障:AF 45 45 FA


## stm32发送说明

数字控件:rin rout gain fhigh

数字控件格式： rin.val=123/xFF/xFF/xFF

曲线控件:s0
文本控件:state reason

state: normal bug
reason: r1open r1short,r2...,r3...,r4...(same to r1)
        c1open c1*2,c2...,c3...(same to c1)


## 故障查表决策规则

优先级从上到下，首个命中即返回。

| 条件 | 故障码 |
|---|---|---|
| `rin > 100k` | **c1open** |
| `dc_out > 2.15` → `9k < rin < 12k` → `g1k < 1` | **r4open** |
| `dc_out > 2.15` → `12k < rin < 15.5k` | **r1open** |
| `dc_out > 2.15` → `rout < 30` → `rin > 2k` | **r3short** |
| `dc_out > 2.15` → `rin < 30` → `rout < 5` | **r1short** |
| `dc_out > 2.15` → `800 < rout < 1000` | **r2short** |
| `9k < rin < 15k` → `g1k > 1` | **c2open** |
| `rin < 300` → `dc_out < 0.02` | **r4short** |
| `rin < 300` → `dc_out < 0.3` | **r3open** |
| `rin < 300` → `0.3 < dc_out < 1.2` | **r2open** |
| 查表不命中 → 扫频: `75k < f_high < 95k` | **c3open** |
| 查表不命中 → 扫频: `f_high ≥ 250k` | **c3x2** |
| 以上都不匹配 | **normal** |

### 符号
- `rin` = r_in_dft (Ω), `rout` = r_out_rms (Ω)
- `g1k` = gain_1k, `g10k` = gain_10k
- `dc_out` = rms_dc_out (V)

### LCD输出
```
state.txt="normal"  或  "bug"
reason.txt="c1open"  / "r1short" / "normal" ...
```