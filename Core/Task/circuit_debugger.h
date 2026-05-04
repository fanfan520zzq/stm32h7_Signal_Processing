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
    FAULT_DC_ONLY
} FaultCode;

typedef struct {
    float     r_in_dft;
    float     r_out_rms;
    float     rms_dc_in;      // 输入端含直流有效值 (V)
    float     rms_dc_out;     // 输出端含直流有效值 (V)
    float     gain_1k;
    float     gain_10k;
    float     f_high;
    FaultCode fault_code;
    uint8_t   dc_only;
    uint8_t   valid;
} CircuitState;

CircuitState Circuit_Learn(void);

#endif
