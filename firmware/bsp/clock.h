#ifndef BSP_CLOCK_H
#define BSP_CLOCK_H

#include <stdint.h>

void bsp_clock_init(void);
uint32_t bsp_clock_get_hz(void);
uint32_t bsp_tick_get(void);
void bsp_delay_ms(uint32_t milliseconds);

#endif