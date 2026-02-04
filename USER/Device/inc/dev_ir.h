#ifndef __DEV_IR_H__
#define __DEV_IR_H__

#include "cw32l010.h"
#include "hal_gpio.h"

#define IR_LEFT_ON              set_gpio_level(CW_GPIOB,GPIO_PIN_5,GPIO_Pin_SET)     
#define IR_LEFT_OFF             set_gpio_level(CW_GPIOB,GPIO_PIN_5,GPIO_Pin_RESET)

#define IR_RIGHT_ON             set_gpio_level(CW_GPIOA,GPIO_PIN_2,GPIO_Pin_SET)   
#define IR_RIGHT_OFF            set_gpio_level(CW_GPIOA,GPIO_PIN_2,GPIO_Pin_RESET)   

#define IR_MIDDLE_LEFT_ON       set_gpio_level(CW_GPIOB,GPIO_PIN_6,GPIO_Pin_SET)   
#define IR_MIDDLE_LEFT_OFF      set_gpio_level(CW_GPIOB,GPIO_PIN_6,GPIO_Pin_RESET)

#define IR_MIDDLE_RIGHT_ON      set_gpio_level(CW_GPIOA,GPIO_PIN_1,GPIO_Pin_SET)       
#define IR_MIDDLE_RIGHT_OFF     set_gpio_level(CW_GPIOA,GPIO_PIN_1,GPIO_Pin_RESET)


void ir_open_all(void);
void ir_close_all(void);

#endif /* __DEV_IR_H__ */
