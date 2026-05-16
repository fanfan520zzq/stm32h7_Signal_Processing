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


## 故障查表决策规则 重构！！！！

优先级从上到下，首个命中即返回。

r_in > 500000 ----c1_open
gain (1.75-2.25) ----c2_open

dc_oc >2.2 : {
        dc_rl (1.95,2.1):{
                r_in>14000 ---- r1_open
                r_in<180 ---- r2_short
                r4_open (9000,14000) ---- r4_open
        }
        dc_rl > 2.15 :{
                gain (12,100) ---- r1_short
                gain<12 ---- r3_short
        }
}

ac_ch2 < 0.05 :{
        dc_oc <0.010 ---- r4_short
        dc_oc (0.03,0.08) ---- r3_open 
        dc_oc (0.8,0.9) ---- r2_open
}

若为normal: {
        FH > 1MHZ ---- c3_open
        FL <200HZ ---- c2*2
        FH (75K , 100KHZ) ---- c3*2
}

### LCD输出
```
state.txt="normal"  或  "bug"
reason.txt="c1open"  / "r1short" / "normal" ...
```