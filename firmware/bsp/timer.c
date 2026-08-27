#include "timer.h"

#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    uint32_t period_ms;
    bsp_timer_callback_t callback;
    void *context;
} bsp_timer_context_t;

static bsp_timer_context_t timer_context;

static void timer_task(void *argument)
{
    bsp_timer_context_t *context = argument;
    TickType_t last_wake = xTaskGetTickCount();
    TickType_t period_ticks = pdMS_TO_TICKS(context->period_ms);

    if (period_ticks == 0U) {
        period_ticks = 1U;
    }

    for (;;) {
        vTaskDelayUntil(&last_wake, period_ticks);
        context->callback(context->context);
    }
}

bsp_timer_status_t bsp_timer_start_periodic(
    uint32_t period_ms,
    bsp_timer_callback_t callback,
    void *context
)
{
    if (period_ms == 0U || callback == 0) {
        return BSP_TIMER_ERROR;
    }

    timer_context.period_ms = period_ms;
    timer_context.callback = callback;
    timer_context.context = context;

    if (xTaskCreate(timer_task, "timer", 256, &timer_context, 2, NULL) != pdPASS) {
        return BSP_TIMER_ERROR;
    }

    return BSP_TIMER_OK;
}