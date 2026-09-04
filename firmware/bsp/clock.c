#include "clock.h"

#include "FreeRTOS.h"
#include "task.h"

static uint32_t system_clock_hz = 20000000U;

#ifdef ROBOT_QEMU
#define CORTEX_M_SYST_CSR (*(volatile uint32_t *) 0xE000E010UL)
#define CORTEX_M_SYST_RVR (*(volatile uint32_t *) 0xE000E014UL)
#define CORTEX_M_SYST_CVR (*(volatile uint32_t *) 0xE000E018UL)
#define CORTEX_M_SYST_CSR_ENABLE (1UL << 0)
#define CORTEX_M_SYST_CSR_TICKINT (1UL << 1)
#define CORTEX_M_SYST_CSR_CLKSOURCE (1UL << 2)
#endif

void bsp_clock_init(void)
{
    system_clock_hz = configCPU_CLOCK_HZ;
#ifdef ROBOT_QEMU
    CORTEX_M_SYST_CSR = 0U;
    CORTEX_M_SYST_RVR = (configCPU_CLOCK_HZ / configTICK_RATE_HZ) - 1U;
    CORTEX_M_SYST_CVR = 0U;
    CORTEX_M_SYST_CSR = CORTEX_M_SYST_CSR_ENABLE
        | CORTEX_M_SYST_CSR_TICKINT | CORTEX_M_SYST_CSR_CLKSOURCE;
#endif
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