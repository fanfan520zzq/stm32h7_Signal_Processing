#ifndef MODULE_STATE_H
#define MODULE_STATE_H

#include <stdint.h>

/**
 * @brief Common state definition for all independent modules
 */
typedef enum {
    MODULE_UNINIT,   /*!< Module not initialized yet */
    MODULE_READY,    /*!< Module initialized and ready for operation */
    MODULE_RUNNING,  /*!< Module is currently running/streaming */
    MODULE_BUSY,     /*!< Module is busy processing data/commands */
    MODULE_ERROR     /*!< Module encountered an error */
} ModuleState_t;

/**
 * @brief Common status structure combining state and error codes
 */
typedef struct {
    ModuleState_t state;
    int32_t error_code;
    uint32_t last_update_tick;
} ModuleStatus_t;

/* Common Error Codes */
#define ERR_OK            0
#define ERR_PARAM        -1
#define ERR_TIMEOUT      -2
#define ERR_BUSY         -3
#define ERR_HARDWARE     -4
#define ERR_NOT_READY    -5

#endif /* MODULE_STATE_H */
