#ifndef VOFA_PROTOCOL_H
#define VOFA_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

// Initialize VOFA (binds to USART1)
void VOFA_Init(void);

// Send up to 4 floats via JustFloat protocol
void VOFA_FireWater(float f1, float f2, float f3, float f4);

// Poll for any incoming debug commands (can parse CMD:DDS_SET or raw binary)
void VOFA_Poll(void);

#endif // VOFA_PROTOCOL_H