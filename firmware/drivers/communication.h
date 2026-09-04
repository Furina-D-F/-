#ifndef ROBOT_COMMUNICATION_H
#define ROBOT_COMMUNICATION_H

#include <stdint.h>

#include "protocol.h"
#include "uart.h"
#include "control.h"

typedef struct {
    robot_uart_rx_ring_t *rx;
    robot_uart_tx_ring_t *tx;
    robot_protocol_parser_t parser;
    uint32_t last_tick;
    uint32_t rx_errors;
    uint32_t duplicate_frames;
    uint32_t handled_frames;
} robot_communication_t;

void robot_communication_init(
    robot_communication_t *communication,
    robot_uart_rx_ring_t *rx,
    robot_uart_tx_ring_t *tx
);
void robot_communication_poll(robot_communication_t *communication, uint32_t tick);
int robot_communication_send_status(
    robot_communication_t *communication,
    uint8_t sequence,
    uint32_t task_counter,
    uint32_t timer_counter
);
void robot_communication_task(void *argument);

#endif