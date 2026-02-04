#include "hal_gpio.h"
#include "cw32l010.h"



/* 调用库函数，设定指定引脚为指定电平 */
void set_gpio_level(GPIO_TypeDef* GPIOx, uint16_t pin,GPIO_PinState level)
{
	GPIO_WritePin(GPIOx,pin,level);
}

void gpio_output_cfg(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	if(pin < GPIO_PIN_All)
	{
		/* 指定特殊的引脚 */
		GPIO_InitStructure.Pins          = pin;
		/* 推挽输出：无外部上拉电阻，内部含有上下拉的MOS管，可以直接输出上下拉高低电平的驱动 */
		GPIO_InitStructure.Mode    = GPIO_MODE_OUTPUT_PP;
		/* 输入模式可以配置GPIO的上升沿触发中断，这里进行关闭 */
		GPIO_InitStructure.IT = GPIO_IT_NONE;
		/* 指定GPIO组，并进行初始化 */
		GPIO_Init(GPIOx, &GPIO_InitStructure);
	}  	
}

void gpio_input_cfg(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	if(pin < GPIO_PIN_All)
	{		
		GPIO_InitStructure.Pins          = pin;
		/* 设置指定引脚为输入模式 */
		GPIO_InitStructure.Mode    = GPIO_MODE_INPUT;
		GPIO_InitStructure.IT = GPIO_IT_NONE;
		GPIO_Init(GPIOx, &GPIO_InitStructure);
	}   
}

uint8_t get_gpio_level(GPIO_TypeDef* GPIOx, uint16_t pin)
{
	return GPIO_ReadPin(GPIOx,pin);
}

