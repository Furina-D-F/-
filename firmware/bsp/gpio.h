#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include <stdint.h>

typedef enum {
    BSP_GPIO_LOW = 0,
    BSP_GPIO_HIGH = 1
} bsp_gpio_level_t;

typedef enum {
    BSP_GPIO_OK = 0,
    BSP_GPIO_INVALID_PIN = -1
} bsp_gpio_status_t;

void bsp_gpio_init(void);
bsp_gpio_status_t bsp_gpio_write(uint32_t pin, bsp_gpio_level_t level);
bsp_gpio_status_t bsp_gpio_toggle(uint32_t pin);
bsp_gpio_status_t bsp_gpio_read(uint32_t pin, bsp_gpio_level_t *level);

#endif