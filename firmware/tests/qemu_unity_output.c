#include <stdint.h>

#define QEMU_UART0_BASE 0x40004000UL
#define QEMU_UART_DR (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x00UL))
#define QEMU_UART_STATE (*(volatile uint32_t *) (QEMU_UART0_BASE + 0x04UL))
#define QEMU_UART_STATE_TXFULL (1UL << 0)

void robot_unity_output_char(int character)
{
    while ((QEMU_UART_STATE & QEMU_UART_STATE_TXFULL) != 0U) {
    }
    QEMU_UART_DR = (uint32_t) (uint8_t) character;
}