#include "uart.h"

void robot_uart_init(robot_uart_rx_ring_t *ring)
{
    ring->head = 0U;
    ring->tail = 0U;
}

robot_uart_result_t robot_uart_rx_isr_push(robot_uart_rx_ring_t *ring, uint8_t byte)
{
    uint16_t next_head = (uint16_t) ((ring->head + 1U) % ROBOT_UART_RX_BUFFER_SIZE);

    if (next_head == ring->tail) {
        return ROBOT_UART_FULL;
    }

    ring->data[ring->head] = byte;
    ring->head = next_head;
    return ROBOT_UART_OK;
}

robot_uart_result_t robot_uart_read(robot_uart_rx_ring_t *ring, uint8_t *byte)
{
    if (byte == 0 || ring->head == ring->tail) {
        return ROBOT_UART_EMPTY;
    }

    *byte = ring->data[ring->tail];
    ring->tail = (uint16_t) ((ring->tail + 1U) % ROBOT_UART_RX_BUFFER_SIZE);
    return ROBOT_UART_OK;
}

uint16_t robot_uart_available(const robot_uart_rx_ring_t *ring)
{
    return ring->head >= ring->tail
        ? (uint16_t) (ring->head - ring->tail)
        : (uint16_t) (ROBOT_UART_RX_BUFFER_SIZE - ring->tail + ring->head);
}

void robot_uart_tx_init(robot_uart_tx_ring_t *ring)
{
    ring->head = 0U;
    ring->tail = 0U;
}

robot_uart_result_t robot_uart_tx_write(robot_uart_tx_ring_t *ring, uint8_t byte)
{
    uint16_t next_head = (uint16_t) ((ring->head + 1U) % ROBOT_UART_TX_BUFFER_SIZE);

    if (next_head == ring->tail) {
        return ROBOT_UART_FULL;
    }

    ring->data[ring->head] = byte;
    ring->head = next_head;
    return ROBOT_UART_OK;
}

robot_uart_result_t robot_uart_tx_read(robot_uart_tx_ring_t *ring, uint8_t *byte)
{
    if (byte == 0 || ring->head == ring->tail) {
        return ROBOT_UART_EMPTY;
    }

    *byte = ring->data[ring->tail];
    ring->tail = (uint16_t) ((ring->tail + 1U) % ROBOT_UART_TX_BUFFER_SIZE);
    return ROBOT_UART_OK;
}

uint16_t robot_uart_tx_available(const robot_uart_tx_ring_t *ring)
{
    return ring->head >= ring->tail
        ? (uint16_t) (ring->head - ring->tail)
        : (uint16_t) (ROBOT_UART_TX_BUFFER_SIZE - ring->tail + ring->head);
}