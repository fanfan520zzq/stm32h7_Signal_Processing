//
// Created by Lenovo on 2026/2/11.
//

#ifndef RTOS0_0_DDS_H
#define RTOS0_0_DDS_H

#include <stdint.h>
#include "protocol.h"

typedef ProtoCmd OP;

typedef struct {
    OP       op;
} APP_Text;

extern uint8_t g_is_adc_continuous;

#endif //RTOS0_0_DDS_H
