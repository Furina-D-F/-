#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include <stdint.h>

typedef void (*bsp_timer_callback_t)(void *context);

typedef enum {
    BSP_TIMER_OK = 0,
    BSP_TIMER_ERROR = -1
} bsp_timer_status_t;

bsp_timer_status_t bsp_timer_start_periodic(
    uint32_t period_ms,
    bsp_timer_callback_t callback,
    void *context
);

#endif