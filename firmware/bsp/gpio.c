#include "gpio.h"

#define BSP_GPIO_PIN_COUNT 16U

static uint32_t gpio_output_register;

void bsp_gpio_init(void)
{
    gpio_output_register = 0U;
}

static int pin_is_valid(uint32_t pin)
{
    return pin < BSP_GPIO_PIN_COUNT;
}

bsp_gpio_status_t bsp_gpio_write(uint32_t pin, bsp_gpio_level_t level)
{
    if (!pin_is_valid(pin)) {
        return BSP_GPIO_INVALID_PIN;
    }

    if (level == BSP_GPIO_HIGH) {
        gpio_output_register |= (1U << pin);
    } else {
        gpio_output_register &= ~(1U << pin);
    }

    return BSP_GPIO_OK;
}

bsp_gpio_status_t bsp_gpio_toggle(uint32_t pin)
{
    if (!pin_is_valid(pin)) {
        return BSP_GPIO_INVALID_PIN;
    }

    gpio_output_register ^= (1U << pin);
    return BSP_GPIO_OK;
}

bsp_gpio_status_t bsp_gpio_read(uint32_t pin, bsp_gpio_level_t *level)
{
    if (!pin_is_valid(pin) || level == 0) {
        return BSP_GPIO_INVALID_PIN;
    }

    *level = (gpio_output_register & (1U << pin)) != 0U
        ? BSP_GPIO_HIGH
        : BSP_GPIO_LOW;
    return BSP_GPIO_OK;
}