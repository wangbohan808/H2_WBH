#include "dev_led.h"
#include "cw32l010.h"
#include "hal_gpio.h"


/* 如果硬件可以封装为多种基本功能，那么便有必要写一个驱动 */

void LED_ON(void)
{
	set_gpio_level(CW_GPIOB,GPIO_PIN_1,GPIO_Pin_RESET);	
}

void LED_OFF(void)
{
	set_gpio_level(CW_GPIOB,GPIO_PIN_1,GPIO_Pin_SET); 
}

/* 占空比控制LED灯亮度：数字量实现模拟量效果*/
void led_set_pwm(uint8_t duty)     
{
	static uint8_t cnt1 = 0;
	cnt1 ++;
	if(cnt1 < duty )
	{
		LED_ON() ;
	}
	else 
	{
		LED_OFF();
		if(cnt1 == 100 )
		{
			cnt1 = 0;
		}
	}
}

/* 呼吸灯效果 */
void led_breath(void)   
{
	static uint16_t time = 0;
	static uint8_t cty = 0;
	static uint8_t breath_mode = 0;
	time ++;
	if(breath_mode == 0)
	{
        /* 调用500次led_breath，改变一次占空比 */
		if(time >= 250)
		{
			time = 0;
			cty ++;
			if(cty >= 100)
			{
				breath_mode = 1;
			}
		}
	}
	else
	{
		if(time >= 250)
		{
			time = 0;
			cty --;
			if(cty == 0)
			{
				breath_mode = 0;
			}
		}
	}
	led_set_pwm(cty);
}


void led_twinkle(void)
{
    /* 静态变量：下面这个流程一直不断循环，快慢由外部的循环速度决定；外部暂停，上一次的状态储存 */
	static uint16_t dispaly_twinkle_count = 0;	
	dispaly_twinkle_count ++ ;
	if(dispaly_twinkle_count <2000 )
	{
		LED_ON();
	}			
	else if(dispaly_twinkle_count>=2000&&dispaly_twinkle_count<=4000) 
	{
		 LED_OFF();
		 if(dispaly_twinkle_count == 4000)
		 {
			 dispaly_twinkle_count = 0;
		 }
	}
}


uint32_t led_twinkle_time = 0;
void set_led_twinkle_time (uint32_t time)
{
    led_twinkle_time = time;
}



