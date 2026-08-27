#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "clock.h"
#include "gpio.h"
#include "timer.h"
#include "communication.h"

volatile uint32_t task_counter;
volatile uint32_t timer_callback_counter;
volatile uint32_t gpio_output_level;
static robot_uart_rx_ring_t communication_rx;
static robot_uart_tx_ring_t communication_tx;
static robot_communication_t communication;

static void gpio_timer_callback(void *context)
{
    bsp_gpio_level_t level;

    (void) context;
    bsp_gpio_toggle(0U);
    bsp_gpio_read(0U, &level);
    gpio_output_level = (uint32_t) level;
    timer_callback_counter++;
}

static void heartbeat_task(void *argument)
{
    (void) argument;

    for (;;) {
        task_counter++;
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void)
{
    bsp_clock_init();
    bsp_gpio_init();
    robot_communication_init(&communication, &communication_rx, &communication_tx);

    if (xTaskCreate(robot_communication_task, "communication", 256,
        &communication, 3, NULL) != pdPASS) {
        for (;;) {
        }
    }

    if (bsp_timer_start_periodic(100U, gpio_timer_callback, NULL) != BSP_TIMER_OK) {
        for (;;) {
        }
    }

    if (xTaskCreate(heartbeat_task, "heartbeat", 256, NULL, 1, NULL) != pdPASS) {
        for (;;) {
        }
    }

    vTaskStartScheduler();

    for (;;) {
    }
}