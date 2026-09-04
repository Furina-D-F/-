#ifndef BSP_QEMU_UART_H
#define BSP_QEMU_UART_H

#include "uart.h"

void bsp_qemu_uart_init(void);
void bsp_qemu_uart_poll_rx(robot_uart_rx_ring_t *rx);
void bsp_qemu_uart_flush_tx(robot_uart_tx_ring_t *tx);

#endif
