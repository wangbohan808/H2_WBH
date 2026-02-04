#include "ir_cfg.h"
#include "cw32l010.h"
#include "hal_gpio.h"
#include "tim_cfg.h"


void ir_output_init(void)
{
    /* 38KHz/50%红外载波输出 */
    tim_ir_pwm_cfg (1266,0);
    
    gpio_output_cfg(CW_GPIOB,GPIO_PIN_5);     
    gpio_output_cfg(CW_GPIOB,GPIO_PIN_6);     
    gpio_output_cfg(CW_GPIOA,GPIO_PIN_1);     
    gpio_output_cfg(CW_GPIOA,GPIO_PIN_2);    
}
