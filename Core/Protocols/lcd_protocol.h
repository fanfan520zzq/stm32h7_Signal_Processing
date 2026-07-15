#ifndef LCD_PROTOCOL_H
#define LCD_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    LCD_EVENT_NONE = 0,
    LCD_EVENT_BTN_START,
    LCD_EVENT_BTN_STOP,
    LCD_EVENT_PARAM_CHANGE
} LCD_Event_t;

typedef struct {
    LCD_Event_t type;
    int32_t value; // For param changes
} LCD_Message_t;

// Init LCD protocol (binds to USART3)
void LCD_Init(void);

// Fetch an event from the LCD (non-blocking)
bool LCD_PollEvent(LCD_Message_t *msg);

// Probe API: Send text to a text control (e.g. t0.txt="Hello")
void LCD_SetText(const char* obj_name, const char* str);

// Probe API: Send number to a number control (e.g. n0.val=123)
void LCD_SetNum(const char* obj_name, int32_t val);

#define DC       1
#define SINE     2
#define SQUARE   3
#define TRIANGLE 4

#define CH1 1
#define CH2 2

// Legacy FFT update hooks
void LCD_Update_Stats(float f1, float v1, uint8_t t1, float f2, float v2, uint8_t t2);
void LCD_Update_Waves(uint8_t type, uint16_t amp, uint8_t ch, float freq);

// Probe API: Switch page (e.g. page 1)
void LCD_SetPage(uint8_t page_id);

#endif // LCD_PROTOCOL_H