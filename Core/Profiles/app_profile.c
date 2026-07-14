#include "app_profile.h"
#include <stdio.h>
#include "si5351.h"

// External module functions
extern void CMD_Init(void);
extern void FFT_Init(void);
extern void UART_Proto_Init(void);
extern void UART_Poll(void);
extern void CMD_Poll(void);

static ProfileType_t current_profile = PROFILE_IDLE;

int32_t App_SelectProfile(ProfileType_t profile) {
    current_profile = profile;
    return ERR_OK;
}

void App_Init(void) {
    if (current_profile == PROFILE_UART_DEBUG) {
        printf("LOG:INFO System Initialized. Profile: UART_DEBUG\r\n");
        UART_Proto_Init();
        CMD_Init();
        FFT_Init();

        // SI5351 Output Test (Stage 3 Verification)
        if (si5351_Init() == 0) { // 0 = ERROR_NONE
            si5351_set_freq(0, 2048000); // 2.048 MHz on CLK0
            printf("LOG:INFO SI5351 initialized, 2.048MHz on CLK0.\r\n");
        } else {
            printf("LOG:ERROR SI5351 init failed (I2C missing?).\r\n");
        }
    } else if (current_profile == PROFILE_IDLE) {
        printf("LOG:INFO System Initialized. Profile: IDLE\r\n");
    }
}

void App_Poll(void) {
    switch (current_profile) {
        case PROFILE_IDLE:
            // Do nothing, safe fallback
            break;

        case PROFILE_UART_DEBUG:
        {
            // ASCII fallback (CMD:PING) and binary VOFA frame check are now inside UART_Poll
            UART_Poll(); // Read binary protocol and ASCII loopback
            CMD_Poll();  // Execute commands
            break;
        }

        default:
            break;
    }
}
