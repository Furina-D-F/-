#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "clock.h"
#include "gpio.h"
#include "qemu_uart.h"
#include "timer.h"
#include "communication.h"
#include "scheduler_validation.h"
#include "robot_tasks.h"

static robot_uart_rx_ring_t communication_rx;
static robot_uart_tx_ring_t communication_tx;
static robot_communication_t communication;

static void gpio_timer_callback(void *context)
{
    (void) context;
    bsp_gpio_toggle(0U);
}

int main(void)
{
    bsp_clock_init();
    bsp_gpio_init();
    robot_communication_init(&communication, &communication_rx, &communication_tx);
    bsp_qemu_uart_init(&communication_rx, &communication_tx);
#ifdef ROBOT_ENABLE_SCHEDULER_VALIDATION
    scheduler_validation_start();
#endif

    if (xTaskCreate(robot_communication_task, "communication", 2048,
        &communication, 3, NULL) != pdPASS) {
        for (;;) {
        }
    }

    if (xTaskCreate(robot_tasks_bootstrap_task, "task_init", 512, NULL,
        4, NULL) != pdPASS) {
        for (;;) {
        }
    }

    if (bsp_timer_start_periodic(100U, gpio_timer_callback, NULL) != BSP_TIMER_OK) {
        for (;;) {
        }
    }

    vTaskStartScheduler();

    for (;;) {
    }
}