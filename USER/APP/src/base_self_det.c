#include "base_self_det.h"
#include "stdint.h"
#include "hal_adc.h"
#include "base_state.h"
#include "dev_ir.h"
#include "tim_cfg.h"
#include "dev_led.h"
#include "func_dust_collect.h"
#include "cw32l010_iwdt.h"
#include "base_dust_mode.h"

void base_self_detect(void)
{
	/* 防止自检函数执行过于频繁 */
	static uint32_t sys_time_cnt = 0;
	if(timer_elapsed(sys_time_cnt)<5)
	{
		return ;
	}
	sys_time_cnt = timer_ms();
    
	/* 尘袋在位检测：高电平不在位，低电平在位 */
	if(GPIO_ReadPin(CW_GPIOA ,GPIO_PIN_0) == 1)
	{
		dust_bag_state= DUST_BAG_STATE_UNSTALL;
        dust_bag_time = 0;
	}
	else
	{
		dust_bag_state = DUST_BAG_STATE_STALL; 
	}
    
	/* 离座计数器：用于判断机器人是否离座，需要连续400次低电平才确认离座 */
	static uint16_t base_undock_counter = 0;
	/* 在座计数器：用于判断机器人是否在座，需要连续400次高电平才确认在座 */
	static uint16_t base_dock_counter = 0;
	
    if(get_charging_cur() < CHARGE_DETECT_CUR_ERROR_RANGE)
    {
		/* 接触不良可能会导致短暂的低电平，需要连续的低电平才会判断为离座 */
		if(base_undock_counter < 400 )
		{
			base_undock_counter ++;
		}
		/* 检测到低电平，重置在座计数器 */
		base_dock_counter = 0;
    }
    else
    {
        /* 有高电平说明可能是在位态，但需要连续的高电平才确认 */
		base_undock_counter = 0;
		/* 连续检测到高电平，在座计数器递增 */
		if(base_dock_counter < 20)
		{
			base_dock_counter++;
		}
    }

    if(base_undock_counter >= 400)
    {
        /* 离座状态 */             
        if(robot_at_dock_state != ROBOT_STATE_NOT_AT_DOCK)
        {
            /* 记录在位状态到不在位状态，转变的时刻 */
            last_undock_time = timer_ms();
            
            robot_at_dock_state = ROBOT_STATE_NOT_AT_DOCK;
            ir_open_all();
        }
    }
    else if(base_dock_counter >= 20)
    {
        /* 在座状态：需要连续20次高电平才确认在座 */
        
        if(robot_at_dock_state != ROBOT_STATE_AT_DOCK)
        {         
            /* 检查是否可以触发集尘（两条件：离座在规定时间内、模式二） */
            uint8_t can_collect = dust_mode_record_undock();
            
            /* 模式二：如果可以触发集尘，设置need_duty标志位 */
            if(can_collect == 1)
            {
                /* 触发集尘：检查尘袋状态和集尘时间 */
                if(dust_bag_state == DUST_BAG_STATE_STALL && 
                   dust_bag_time <= 216000*1000)
                {
                    need_duty = 1;
                }
            }
            
            robot_at_dock_state = ROBOT_STATE_AT_DOCK;
            last_undock_time = 0;
        }
    }
	/* 在自检函数进行喂狗操作 */
    IWDT_Refresh();   
}
