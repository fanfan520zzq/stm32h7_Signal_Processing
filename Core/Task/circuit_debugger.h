//
// circuit_debugger.h — 电路故障检测与学习
//

#ifndef CIRCUIT_DEBUGGER_H
#define CIRCUIT_DEBUGGER_H

#include "main.h"
#include <stdint.h>

typedef enum {
    FAULT_NONE = 0,
    FAULT_C1_OPEN,
    FAULT_R1_OPEN,
    FAULT_R2_OPEN,
    FAULT_R3_OPEN,
    FAULT_R4_OPEN,
    FAULT_C2_OPEN,
    FAULT_R1_SHORT,
    FAULT_R2_SHORT,
    FAULT_R3_SHORT,
    FAULT_R4_SHORT,
    FAULT_C3_OPEN,
    FAULT_C3_x2,
    FAULT_DC_ONLY
} FaultCode;

typedef struct {
    float     rms_ac_ch1;      // CH1 AC有效值@1kHz, ADC端原始V
    float     rms_ac_ch2;      // CH2 AC有效值@1kHz, ADC端原始V
    float     rms_dc_oc;       // 输出端直流有效值，空载
    float     rms_dc_ld;       // 输出端直流有效值，带载
    float     r_in_dft;
    float     r_out_rms;
    float     rms_dc_in;       // 输入端含直流有效值 (V)
    float     gain_1k;
    float     f_low;           // -3dB下限截止频率
    float     f_high;          // -3dB上限截止频率
    FaultCode fault_code;
    uint8_t   dc_only;
    uint8_t   valid;
} CircuitState;

CircuitState Circuit_Learn(void);

#endif
