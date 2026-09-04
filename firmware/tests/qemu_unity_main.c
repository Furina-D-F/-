#include "FreeRTOS.h"
#include "task.h"

#include "clock.h"
#include "qemu_uart.h"

int robot_unity_run_all(void);

static void unity_task(void *argument)
{
    (void) argument;
    (void) robot_unity_run_all();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

int main(void)
{
    bsp_clock_init();
    bsp_qemu_uart_init();
    if (xTaskCreate(unity_task, "unity", 1024, NULL, 3, NULL) != pdPASS) {
        for (;;) {
        }
    }
    vTaskStartScheduler();
    for (;;) {
    }
}