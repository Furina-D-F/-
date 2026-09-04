#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "communication.h"

static TickType_t host_tick;

TickType_t xTaskGetTickCount(void)
{
    return host_tick;
}

void vTaskDelay(const TickType_t ticks)
{
    (void) ticks;
}

static int flush_tx(robot_uart_tx_ring_t *tx)
{
    uint8_t byte;

    while (robot_uart_tx_read(tx, &byte) == ROBOT_UART_OK) {
        if (fwrite(&byte, 1U, 1U, stdout) != 1U) {
            return 1;
        }
    }
    fflush(stdout);
    return 0;
}

int main(void)
{
    robot_uart_rx_ring_t rx;
    robot_uart_tx_ring_t tx;
    robot_communication_t communication;
    struct timeval timeout;
    fd_set input_set;

    host_tick = 0U;
    robot_communication_init(&communication, &rx, &tx);

    for (;;) {
        FD_ZERO(&input_set);
        FD_SET(STDIN_FILENO, &input_set);
        timeout.tv_sec = 0;
        timeout.tv_usec = 10000;
        int ready = select(STDIN_FILENO + 1, &input_set, NULL, NULL, &timeout);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            return 1;
        }
        if (ready == 0) {
            host_tick += pdMS_TO_TICKS(10U);
            robot_communication_poll(&communication, host_tick);
            robot_control_update(0.01f);
            continue;
        }

        uint8_t input;
        ssize_t bytes_read = read(STDIN_FILENO, &input, sizeof(input));
        if (bytes_read == 0) {
            return 0;
        }
        if (bytes_read < 0) {
            return errno == EINTR ? 0 : 1;
        }

        if (robot_uart_rx_isr_push(&rx, input) != ROBOT_UART_OK) {
            return 1;
        }

        robot_communication_poll(&communication, host_tick);
        if (flush_tx(&tx) != 0) {
            return 1;
        }
    }
}