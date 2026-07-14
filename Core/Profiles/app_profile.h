#ifndef APP_PROFILE_H
#define APP_PROFILE_H

#include "module_state.h"

/**
 * @brief Available System Profiles
 */
typedef enum {
    PROFILE_IDLE = 0,
    PROFILE_UART_DEBUG,
    PROFILE_ADC_VOFA,
    PROFILE_DAC_DDS,
    PROFILE_SIGNAL_ANALYSIS
} ProfileType_t;

/**
 * @brief Initialize the application with a specific profile
 * @param profile Profile to activate
 * @return int32_t ERR_OK on success
 */
int32_t App_SelectProfile(ProfileType_t profile);

/**
 * @brief Initialize resources required by current profile
 */
void App_Init(void);

/**
 * @brief Main polling loop for current profile
 */
void App_Poll(void);

extern volatile uint8_t test_adc_flag;
extern volatile uint32_t test_adc_len;

extern volatile uint8_t  test_dds_flag;
extern volatile uint8_t  test_dds_wave;
extern volatile uint32_t test_dds_freq;
extern volatile uint16_t test_dds_vpp;
extern volatile uint16_t test_dds_bias;
extern volatile uint8_t  test_dds_duty;

#endif /* APP_PROFILE_H */
