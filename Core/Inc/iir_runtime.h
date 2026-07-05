#ifndef IIR_RUNTIME_H
#define IIR_RUNTIME_H

#include "main.h"

extern volatile uint8_t g_iir_rt_active;

void iir_rt_start_current_bpf(void);
void iir_rt_start_passthrough(void);
void iir_rt_stop(void);
void iir_rt_process_half(int half_idx);
void iir_rt_print_stats(void);

#endif // IIR_RUNTIME_H
