#include "qemu_uart.h"

#include <stdint.h>

#define QEMU_UART0_BASE 0x40004000UL
#define QEMU_UART_DR (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x00UL))
#define QEMU_UART_STATE (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x04UL))
#define QEMU_UART_CTRL (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x08UL))
#define QEMU_UART_INTSTATUS (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x0CUL))
#define QEMU_UART_BAUDDIV (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x10UL))

#define QEMU_UART_STATE_TXFULL (1UL << 0)
#define QEMU_UART_STATE_RXFULL (1UL << 1)
#define QEMU_UART_CTRL_TXEN (1UL << 0)
#define QEMU_UART_CTRL_RXEN (1UL << 1)
#define QEMU_UART_CTRL_TXIRQEN (1UL << 2)
#define QEMU_UART_CTRL_RXIRQEN (1UL << 3)
#define QEMU_UART_INT_TX (1UL << 0)
#define QEMU_UART_INT_RX (1UL << 1)
#define CORTEX_M_NVIC_ISER0 (*(volatile uint32_t *) 0xE000E100UL)
#define QEMU_UART_RX_IRQ_NUMBER 0U
#define QEMU_UART_TX_IRQ_NUMBER 1U

static robot_uart_rx_ring_t *uart_rx;
static robot_uart_tx_ring_t *uart_tx;

void bsp_qemu_uart_init(robot_uart_rx_ring_t *rx, robot_uart_tx_ring_t *tx)
{
    uart_rx = rx;
    uart_tx = tx;
    QEMU_UART_CTRL = 0U;
    QEMU_UART_BAUDDIV = 173U;
    QEMU_UART_CTRL = QEMU_UART_CTRL_TXEN | QEMU_UART_CTRL_RXEN
        | (rx != 0 ? QEMU_UART_CTRL_RXIRQEN : 0U);
    CORTEX_M_NVIC_ISER0 = 1UL << QEMU_UART_RX_IRQ_NUMBER;
}

void bsp_qemu_uart_flush_tx(void)
{

    QEMU_UART_CTRL &= ~QEMU_UART_CTRL_TXIRQEN;
    bsp_qemu_uart_service();
}

void bsp_qemu_uart_irq_handler(void)
{
    uint8_t byte;
    uint32_t interrupt_status = QEMU_UART_INTSTATUS;

    if (uart_rx != 0 && (interrupt_status & QEMU_UART_INT_RX) != 0U) {
        while ((QEMU_UART_STATE & QEMU_UART_STATE_RXFULL) != 0U) {
            byte = (uint8_t) QEMU_UART_DR;
            (void) robot_uart_rx_isr_push(uart_rx, byte);
        }
        QEMU_UART_INTSTATUS = QEMU_UART_INT_RX;
    }

    if ((interrupt_status & QEMU_UART_INT_TX) != 0U) {
        QEMU_UART_INTSTATUS = QEMU_UART_INT_TX;
    }
}

void bsp_qemu_uart_service(void)
{
    uint8_t byte;

    if (uart_rx != 0) {
        while ((QEMU_UART_STATE & QEMU_UART_STATE_RXFULL) != 0U) {
            byte = (uint8_t) QEMU_UART_DR;
            (void) robot_uart_rx_isr_push(uart_rx, byte);
        }
    }
    if (uart_tx != 0) {
        while ((QEMU_UART_STATE & QEMU_UART_STATE_TXFULL) == 0U
            && robot_uart_tx_read(uart_tx, &byte) == ROBOT_UART_OK) {
            QEMU_UART_DR = byte;
        }
    }
}
