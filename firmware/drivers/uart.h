#ifndef ROBOT_UART_H
#define ROBOT_UART_H

#include <stdint.h>

#define ROBOT_UART_RX_BUFFER_SIZE 256U
#define ROBOT_UART_TX_BUFFER_SIZE 256U

typedef enum {
    ROBOT_UART_OK = 0,
    ROBOT_UART_EMPTY = 1,
    ROBOT_UART_FULL = -1
} robot_uart_result_t;

typedef struct {
    uint8_t data[ROBOT_UART_RX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} robot_uart_rx_ring_t;

typedef struct {
    uint8_t data[ROBOT_UART_TX_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
} robot_uart_tx_ring_t;

void robot_uart_init(robot_uart_rx_ring_t *ring);
robot_uart_result_t robot_uart_rx_isr_push(robot_uart_rx_ring_t *ring, uint8_t byte);
robot_uart_result_t robot_uart_read(robot_uart_rx_ring_t *ring, uint8_t *byte);
uint16_t robot_uart_available(const robot_uart_rx_ring_t *ring);

void robot_uart_tx_init(robot_uart_tx_ring_t *ring);
robot_uart_result_t robot_uart_tx_write(robot_uart_tx_ring_t *ring, uint8_t byte);
robot_uart_result_t robot_uart_tx_read(robot_uart_tx_ring_t *ring, uint8_t *byte);
uint16_t robot_uart_tx_available(const robot_uart_tx_ring_t *ring);

#endif