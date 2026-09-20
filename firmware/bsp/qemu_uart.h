#ifndef BSP_QEMU_UART_H
#define BSP_QEMU_UART_H

#include "uart.h"

void bsp_qemu_uart_init(robot_uart_rx_ring_t *rx, robot_uart_tx_ring_t *tx);
void bsp_qemu_uart_flush_tx(void);
void bsp_qemu_uart_irq_handler(void);
void bsp_qemu_uart_service(void);

#endif
