#include "lcd_protocol.h"
#include "usart_driver.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

void LCD_Init(void) {
}

// Send standard tail for USART HMI displays (0xFF 0xFF 0xFF)
static void LCD_SendTail(void) {
    uint8_t tail[3] = {0xFF, 0xFF, 0xFF};
    USART_Driver_WriteBytes(&huart1, tail, 3);
}

void LCD_SetText(const char* obj_name, const char* str) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj_name, str);
    if (len > 0) {
        USART_Driver_WriteBytes(&huart1, (uint8_t*)buf, len);
        LCD_SendTail();
    }
}

void LCD_SetNum(const char* obj_name, int32_t val) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%s.val=%ld", obj_name, val);
    if (len > 0) {
        USART_Driver_WriteBytes(&huart1, (uint8_t*)buf, len);
        LCD_SendTail();
    }
}

void LCD_SetPage(uint8_t page_id) {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "page %d", page_id);
    if (len > 0) {
        USART_Driver_WriteBytes(&huart1, (uint8_t*)buf, len);
        LCD_SendTail();
    }
}

void LCD_AddWavePoint(uint8_t obj_id, uint8_t ch, uint8_t val) {
    char buf[32];
    int len = snprintf(buf, sizeof(buf), "addt %d,%d,%d", obj_id, ch, val);
    if (len > 0) {
        USART_Driver_WriteBytes(&huart1, (uint8_t*)buf, len);
        LCD_SendTail();
    }
}

// Polls LCD events from USART1.
// Format example: "START\n", "STOP\n", "PARAM:1234\n"
bool LCD_PollEvent(LCD_Message_t *msg) {
    uint8_t byte;
    static char rx_buf[64];
    static uint8_t idx = 0;
    
    while (USART_Driver_ReadByte(&huart1, &byte)) {
        if (byte == '\n' || byte == '\r') {
            if (idx > 0) {
                rx_buf[idx] = '\0';
                idx = 0;
                
                if (strcmp(rx_buf, "START") == 0) {
                    msg->type = LCD_EVENT_BTN_START;
                    return true;
                } else if (strcmp(rx_buf, "STOP") == 0) {
                    msg->type = LCD_EVENT_BTN_STOP;
                    return true;
                } else if (strncmp(rx_buf, "PARAM:", 6) == 0) {
                    msg->type = LCD_EVENT_PARAM_CHANGE;
                    msg->value = atoi(&rx_buf[6]);
                    return true;
                }
            }
        } else {
            if (idx < sizeof(rx_buf) - 1) {
                rx_buf[idx++] = (char)byte;
            }
        }
    }
    return false;
}

void LCD_Update_Stats(float f1, float v1, uint8_t t1, float f2, float v2, uint8_t t2) {
    // Stub for legacy FFT module
    (void)f1; (void)v1; (void)t1;
    (void)f2; (void)v2; (void)t2;
}

void LCD_Update_Waves(uint8_t type, uint16_t amp, uint8_t ch, float freq) {
    // Stub for legacy FFT module
    (void)type; (void)amp; (void)ch; (void)freq;
}