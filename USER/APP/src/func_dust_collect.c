#include "func_dust_collect.h"
#include <stdint.h>
#include "tim_cfg.h"
#include "cw32l010.h"
#include "hal_gpio.h"
#include <stdlib.h>
#include "base_state.h"
#include "func_ir_ack.h"
#include "hal_uart.h"

uint8_t ac_frequency = 0;
/*此变量用于过零中断触发标志*/
uint8_t vacuum_zero_check_flg =  0;

void ac_frequency_calculate(void)
{
	static uint8_t filtering_flg = 0;
    static int ac_time_start = 0;
    static int ac_time_elapsed = 0;
    static uint64_t systick_flag = 0;
    
    
	if(ac_frequency == 0)
	{
		/* 标志位控制状态机的运行，两个case之间的时间差决定频率 */
		switch(filtering_flg)
		{
			case 0:
				ac_time_start = timer_ms();
				filtering_flg ++;
				systick_flag = timer_ms();
			break;

			case 1:
				if(timer_elapsed(systick_flag) > 5)
				{
					systick_flag = timer_ms();
					ac_time_elapsed = timer_elapsed(ac_time_start);
					if(abs(ac_time_elapsed-10)<1)
					{
						ac_frequency = 50;
					}
					else if(abs(ac_time_elapsed-8)<1)
					{
						ac_frequency=60;
					}
					else
					{
						filtering_flg = 0;
					}
				}
			break;
		}
	}
	else
	{
		/* 第一个过零点之后无缝接入，集尘标志位置一，只进入一次 */
		if(filtering_flg == 1)
		{
			filtering_flg = 0;
			systick_flag = timer_ms();
			vacuum_zero_check_flg = 1 ;
		}
		else
		{
			if(timer_elapsed(systick_flag) > 5)
			{
				systick_flag = timer_ms();
				vacuum_zero_check_flg = 1;			
			}
		}
	}
}

/* 集尘完成时间,用于产测模式限定短时间内不重复集尘 */
uint32_t time_for_dust_finish = 0;
uint32_t get_time_for_dust_finish(void)
{
    return timer_elapsed(time_for_dust_finish);
}

/* 此变量用于设置集尘持续时间 */
uint8_t dust_absorption_time = 10;
/* 此变量用于判断是否需要集尘，置零关闭集尘的风机 */
uint8_t need_duty = 0;
/* 记录此次集尘发生的时间，用于判断何时集尘结束 */
uint64_t dust_collect_time_cnt = 0;
/* 全局变量累计记录：此次更换尘袋后，集尘过程的累计时间 */
uint32_t dust_bag_time = 0;
void dust_absorption_ctrl(void)
{
    static uint16_t rising_time = 0;
    static uint16_t zero_check_cnt = 0;
    /*为了让用户能听清楚"开始集尘"语音再开始集尘,用此变量作为标识*/
    static uint8_t collect_flag = 0;   

    if(dust_collect_time_cnt < 1)
    {
        /* 获取集尘开始时间 1.标志集尘流程开始 2.用于后续超时判断 */
        dust_collect_time_cnt = timer_ms();
        if(ac_frequency == 50)
        {
            rising_time = 600 ;
        }
        else if(ac_frequency == 60)
        {
            rising_time = 670 ;
        }
        vacuum_zero_check_flg = 0;
    }
  
    if(collect_flag == 0 && timer_elapsed(dust_collect_time_cnt) > 1 * 1000)
    {
        collect_flag = 1;
    }

    /* 达到集尘时间，将标志位置零 */
    if(timer_elapsed(dust_collect_time_cnt) > (dust_absorption_time + 1) * 1000)
    {
        need_duty = 0;
        time_for_dust_finish = timer_ms();
        
        if(base_work_mode == BASE_WORK_MODE_NORMAL)
        {
            sent_buff[0]=0x0e;
            sent_buff[1]=0x01;
            sent_buff[2]=0x0f;	
            set_ir_sent_bite(3);	
            set_ir_send_count(3);      		
        }
        else           //产测模式集尘结束
        {
            sent_buff[0]=0xaa;
            sent_buff[1]=0x01;
            sent_buff[2]=0x05;
            sent_buff[3]=0x01;
            sent_buff[4]=sent_buff[0] +sent_buff[1]+sent_buff[2]+sent_buff[3];
            set_ir_sent_bite(5);	
            set_ir_send_count(3);      				
        }
	}	

    /* 标志位置零，风机关闭 */
    if(need_duty < 1)
    {
        dust_bag_time = dust_bag_time + timer_elapsed(dust_collect_time_cnt) ;
        dust_collect_time_cnt = 0;
        collect_flag = 0;
        set_gpio_level(CW_GPIOB,GPIO_PIN_4,GPIO_Pin_RESET);
    }

    /* 检测到了零点：风机引脚的启动与缓启动 */
    if(vacuum_zero_check_flg == 1 && collect_flag == 1)
    {
        vacuum_zero_check_flg = 0;
        
        /* 配置缓启动阶段引脚置零的时间占空比 */
        zero_check_cnt ++;
        if((zero_check_cnt % 2) == 0) 
        {
            if(rising_time >215)          
            {
                if(ac_frequency == 50)
                {
                    rising_time -= 3;
                }
                else if(ac_frequency == 60)
                {
                    rising_time -= 2;
                }
            }
            else
            {
                rising_time = 215;
            }
        }
        
        /* 实际控制风机转动部分 */
        if(rising_time > 215)
        {
            /* 初期引脚间歇通电,防止交流风机刚启动功率就非常大 */
            delay_10us(rising_time);				
            set_gpio_level(CW_GPIOB,GPIO_PIN_4,GPIO_Pin_SET);
            delay_10us(10);
            set_gpio_level(CW_GPIOB,GPIO_PIN_4,GPIO_Pin_RESET);	
        }
        else
        {
            /* 后期功率全开 */
            set_gpio_level(CW_GPIOB,GPIO_PIN_4,GPIO_Pin_SET);
        }	

    }   
}

/* 为什么循环4次就是1us */
static void delay_us(uint16_t us)
{
    uint16_t i = 0;

    while(us--)
    {
        i = 4;			
        while(i--);
    }
}

void delay_10us(uint16_t us)
{
    while(us--)
    {
        delay_us(10);
    }
}

