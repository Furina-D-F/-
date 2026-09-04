#include "qemu_uart.h"

#include <stdint.h>

#define QEMU_UART0_BASE 0x40004000UL
#define QEMU_UART_DR (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x00UL))
#define QEMU_UART_STATE (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x04UL))
#define QEMU_UART_CTRL (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x08UL))
#define QEMU_UART_BAUDDIV (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x10UL))

#define QEMU_UART_STATE_TXFULL (1UL << 0)
#define QEMU_UART_STATE_RXFULL (1UL << 1)
#define QEMU_UART_CTRL_TXEN (1UL << 0)
#define QEMU_UART_CTRL_RXEN (1UL << 1)

void bsp_qemu_uart_init(void)
{
    QEMU_UART_CTRL = 0U;
    QEMU_UART_BAUDDIV = 173U;
    QEMU_UART_CTRL = QEMU_UART_CTRL_TXEN | QEMU_UART_CTRL_RXEN;
}

void bsp_qemu_uart_poll_rx(robot_uart_rx_ring_t *rx)
{
    while ((QEMU_UART_STATE & QEMU_UART_STATE_RXFULL) != 0U) {
        if (robot_uart_rx_isr_push(rx, (uint8_t) QEMU_UART_DR) != ROBOT_UART_OK) {
            break;
        }
    }
}

void bsp_qemu_uart_flush_tx(robot_uart_tx_ring_t *tx)
{
    uint8_t byte;

    while ((QEMU_UART_STATE & QEMU_UART_STATE_TXFULL) == 0U
        && robot_uart_tx_read(tx, &byte) == ROBOT_UART_OK) {
        QEMU_UART_DR = byte;
    }
}
