#include "fan_cfg.h"
#include "hal_gpio.h"


void fan_gpio_init(void)
{
    gpio_output_cfg(CW_GPIOB,GPIO_PIN_4);
    set_gpio_level(CW_GPIOB,GPIO_PIN_4,GPIO_Pin_RESET);
}

