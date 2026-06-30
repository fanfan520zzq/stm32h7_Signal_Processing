#ifndef ADC_SYNC_H
#define ADC_SYNC_H

#include "main.h"

extern uint32_t g_dma_buf[];
extern volatile uint8_t g_acq_done;

void adc_sync_init(void);
void adc_set_resolution_time(uint32_t resolution, uint32_t sampling_time);
void acq_start_window(int len);
void acq_get_window(uint16_t* ch2, uint16_t* ch3, int len);

#endif // ADC_SYNC_H
