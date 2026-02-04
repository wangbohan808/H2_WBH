#ifndef __HAL_GPIO_H
#define __HAL_GPIO_H

#include "cw32l010.h"
#include "cw32l010_gpio.h"

void gpio_output_cfg(GPIO_TypeDef* GPIOx, uint16_t pin);
void gpio_input_cfg(GPIO_TypeDef* GPIOx, uint16_t pin);
void set_gpio_level(GPIO_TypeDef* GPIOx, uint16_t pin,GPIO_PinState level);
uint8_t get_gpio_level(GPIO_TypeDef* GPIOx, uint16_t pin);


#endif /* __HAL_GPIO_H */
