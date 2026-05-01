## stm32接收命令

强迫复位：AF 01 01 FA 
1khz下输入电阻，输出电阻，增益一键测量:AF 12 12 FA
幅频特性测量:AF 23 23 FA
学习当前电路:AF 34 34 FA
测量当前故障:AF 45 45 FA


## stm32发送说明

数字控件:rin rout gain fhigh
曲线控件:s0
文本控件:state reason

state: normal bug
reason: r1open r1short,r2...,r3...,r4...(same to r1)
        c1open c1*2,c2...,c3...(same to c1)

