#include "tim_cfg.h"
#include "cw32l010.h"
#include <stdint.h>
#include "cw32l010_sysctrl.h"
#include "cw32l010_gtim.h"
#include "cw32l010_gpio.h"
#include "cw32l010_atim.h"
#include "base_state.h"
#include "func_docking_guide.h"
#include "func_pole_decoder.h"
#include "dev_ir.h"
#include "func_ir_ack.h"
#include "dev_led.h"
/***
 此函数不断输出方波
 arr：自动重装载值，影响一个周期的计数值 psc：预分频值，影响每一个时钟周期信号的时间 
 此函数输出方波信号，设置pwm信号一个周期有多少个时钟脉冲，每个时钟脉冲的周期 
***/
void tim_ir_pwm_cfg(uint16_t arr,uint16_t psc)
{
	/* 使能使用的GPIOA引脚组时钟，使能使用的GTIM定时器 */
	__SYSCTRL_GPIOA_CLK_ENABLE();
	__SYSCTRL_GTIM1_CLK_ENABLE();

	GTIM_InitTypeDef GTIM_InitStruct = {0};
	GTIM_OCModeCfgTypeDef GTIM_OCModeCfgStruct = {DISABLE,DISABLE,0};

	/* 配置PA5引脚对应的各个寄存器：使能数字功能（禁用模拟功能）、设置为输出模式、推挽模式、将引脚复用为GTIM的CH2 */
	PA05_DIGTAL_ENABLE();
	PA05_DIR_OUTPUT();
	PA05_PUSHPULL_ENABLE();
	PA05_AFx_GTIM1CH2();

	/* 计数器采用边沿对齐模式计数：0到ARR，与中心对齐计数对应 */
	GTIM_InitStruct.AlignMode = GTIM_ALIGN_MODE_EDGE;
	/* 开启ARR寄存器的影子寄存器，防止影响此次的计数周期 */
	GTIM_InitStruct.ARRBuffState = GTIM_ARR_BUFF_EN;
	/* 在边沿对齐的基础上设置为向上计数 */
	GTIM_InitStruct.Direction = GTIM_DIRECTION_UP;
	GTIM_InitStruct.EventOption = GTIM_EVENT_NORMAL;
	/* 设置预分频器：实际计数频率 = 系统时钟 / (psc + 1) */
	GTIM_InitStruct.Prescaler = psc;
	GTIM_InitStruct.PulseMode = GTIM_PULSE_MODE_DIS;
	/* 设置自动重装载值：PWM频率 = 系统时钟 / [(psc+1) × (arr+1)] */
	GTIM_InitStruct.ReloadValue = arr;
	GTIM_InitStruct.UpdateOption = GTIM_UPDATE_DIS;
	GTIM_TimeBaseInit(CW_GTIM1, &GTIM_InitStruct);


	/* PWM的设置还需要配置输出比较模式 */
	GTIM_OCModeCfgStruct.FastMode = DISABLE;
	/* 设置输出比较的模式为PWM模式1：CNT < CCR时输出有效电平，CNT ≥ CCR时输出无效电平 */
	GTIM_OCModeCfgStruct.OCMode = GTIM_OC_MODE_PWM1;
	/* 非反相极性（高电平有效）：反相会改变占空比含义 */
	GTIM_OCModeCfgStruct.OCPolarity = GTIM_OC_POLAR_NONINVERT;
	GTIM_OCModeCfgStruct.PreloadState = DISABLE;
	GTIM_OC2ModeCfg(CW_GTIM1, &GTIM_OCModeCfgStruct);

	/* 设置占空比 */
	GTIM_SetCompare2(CW_GTIM1, arr/2);
	/* 使能使用的CH2 */
	GTIM_OC2Cmd(CW_GTIM1, ENABLE);
	/* 启动定时器 */
	GTIM_Cmd(CW_GTIM1, ENABLE);
}


/* ==================== 间隔100us触发一次中断回调 ==================== */


/* 使能高级定时器的中断 */
void NVIC_ConfigurationATIM(void)
{
	__disable_irq();
	NVIC_EnableIRQ(ATIM_IRQn);
	__enable_irq();
}

/* 此函数用于计数，定时触发回调函数 */
void tim_100us_irq_cfg(uint16_t arr,uint16_t psc)
{
	/* 使能时钟，使得后续可以配置ATIM的寄存器；开启ATIM的NVIC，使得ATIM产生的中断可以被CPU处理 */
	__SYSCTRL_ATIM_CLK_ENABLE();
	NVIC_ConfigurationATIM() ;
	ATIM_InitTypeDef ATIM_InitStruct = {DISABLE,0};

	/* 仅仅配置计数器，使之不断进行循环计数 */
	ATIM_InitStruct.BufferState = ENABLE;                  
	ATIM_InitStruct.CounterAlignedMode = ATIM_COUNT_ALIGN_MODE_EDGE;   
	ATIM_InitStruct.CounterDirection = ATIM_COUNTING_UP;        
	ATIM_InitStruct.CounterOPMode = ATIM_OP_MODE_REPETITIVE;    
	ATIM_InitStruct.Prescaler = psc;                   
	ATIM_InitStruct.ReloadValue = arr;                 
	ATIM_InitStruct.RepetitionCounter = 0;            

	/* 将上述配置加载进去，启用高级定时器 */
	ATIM_Init(&ATIM_InitStruct);
	/* 使能计数器更新中断：计数器溢出时调用ATIM_IRQHandler中断回调函数 */
	ATIM_ITConfig(ATIM_IT_UIE, ENABLE);            
	/* 启动定时器，让计数器工作 */
	ATIM_Cmd(ENABLE);
}



/* 间隔100us触发一次中断回调 */
void ATIM_IRQHandler(void)
{
    if (ATIM_GetITStatus(ATIM_STATE_UIF))
    {
        ATIM_ClearITPendingBit(ATIM_STATE_UIF);

        
        /* 正常模式、机器人不在座，才启动发送红外引导码功能 */
        if(base_work_mode == BASE_WORK_MODE_NORMAL && robot_at_dock_state == ROBOT_STATE_NOT_AT_DOCK)
        {
            ir_docking_guide_send();
        }
        else
        {
            pole_detect_capture();
            ir_send_ack();
        }
        
        /* 正常模式灯显可以直接确定，产测模式需要灵活调整；所以三个状态确定后，直接确定正常模式的灯显 */
        if(base_work_mode == BASE_WORK_MODE_NORMAL)
        {
            /* 正常模式下，尘袋不在位灯显一定是闪烁 */
            if(dust_bag_state == DUST_BAG_STATE_UNSTALL)
            {
                led_twinkle();
            }
            else
            {
                /* 最后一级判断机器是否在位 */
                if(robot_at_dock_state == ROBOT_STATE_AT_DOCK)
                {
                    led_breath();
                }
                else
                {
                    LED_ON();
                }
            }
        }
    }
}

/* ==================== 间隔1ms触发一次中断回调 ==================== */
uint64_t tick_ms_num = 0;
void SysTick_Handler(void)
{
    tick_ms_num++;
    
    /* 消耗led_twinkle_time的一个数值，执行led_twinkle函数内部的一个步骤 */
    if(led_twinkle_time > 0)
    {
        led_twinkle_time--;
        led_twinkle();
    }
    else
    {
        led_twinkle_time = 0;
    }
}

uint64_t timer_ms(void)
{
	return tick_ms_num;
}

uint64_t timer_elapsed(uint64_t num)
{
	return (tick_ms_num - num);
}                     
