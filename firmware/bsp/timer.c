#include "timer.h"

#include "FreeRTOS.h"

typedef struct {
    TickType_t period_ticks;
    TickType_t elapsed_ticks;
    bsp_timer_callback_t callback;
    void *context;
    BaseType_t active;
} bsp_timer_context_t;

static bsp_timer_context_t timer_context;

__attribute__((weak)) void robot_tasks_tick_isr(void)
{
}

void vApplicationTickHook(void)
{
    robot_tasks_tick_isr();
    if (timer_context.active == pdFALSE) {
        return;
    }

    timer_context.elapsed_ticks++;
    if (timer_context.elapsed_ticks >= timer_context.period_ticks) {
        timer_context.elapsed_ticks = 0U;
        timer_context.callback(timer_context.context);
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

    timer_context.period_ticks = pdMS_TO_TICKS(period_ms);
    if (timer_context.period_ticks == 0U) {
        timer_context.period_ticks = 1U;
    }
    timer_context.elapsed_ticks = 0U;
    timer_context.callback = callback;
    timer_context.context = context;
    timer_context.active = pdTRUE;

    return BSP_TIMER_OK;
}