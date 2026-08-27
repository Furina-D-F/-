#include "clock.h"

#include "FreeRTOS.h"
#include "task.h"

static uint32_t system_clock_hz = 20000000U;

void bsp_clock_init(void)
{
    system_clock_hz = configCPU_CLOCK_HZ;
}

uint32_t bsp_clock_get_hz(void)
{
    return system_clock_hz;
}

uint32_t bsp_tick_get(void)
{
    return (uint32_t) xTaskGetTickCount();
}

void bsp_delay_ms(uint32_t milliseconds)
{
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}