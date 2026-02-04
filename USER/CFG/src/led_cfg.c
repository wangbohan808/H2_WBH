#include "led_cfg.h"
#include "cw32l010.h"
#include "hal_gpio.h"
#include "dev_led.h"

void led_init(void)
{
  gpio_output_cfg(CW_GPIOB,GPIO_PIN_1);
  LED_OFF();            
}
