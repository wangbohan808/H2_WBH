#include "func_pole_decoder.h"
#include <stdint.h>
#include "utils_queue.h"
#include "hal_adc.h"
#include "base_state.h"
#include "func_dust_collect.h"
#include "func_ir_ack.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "dev_led.h"
#include "tim_cfg.h"
#include "hal_uart.h"
#include "uart_cfg.h"
#include "base_dust_mode.h"

/* 接收一个字节的值的储存中间变量 */
static uint8_t pole_working_byte;
/* 接收一个字节（8位）的计数器 */
static uint8_t pole_capture_counter;

/* 储存原始接收数据的队列 */
uint8_t pole_rx_buffer[POLE_QUEUE_LEN] = {0};
static queue_circular_t pole_rx_queue;
static uint8_t pole_rx_queue_init_flag = 0;  // 队列初始化标志

/* 解码器状态结构体 */
pole_decode_t pole_decode = {POLE_RESYNC, 0};

/* 各状态持续时间统计变量（单位：0.1ms） */
uint32_t pole_resync_time = 0;                    // POLE_RESYNC 状态持续时间
uint32_t pole_high_time = 0;                      // POLE_HIGH 状态持续时间
uint32_t pole_low_time = 0;                       // POLE_LOW 状态持续时间
uint32_t pole_dust_collect_high_time = 0;         // POLE_DUST_COLLECT_HIGH 状态持续时间
uint32_t pole_dust_collect_low_time = 0;          // POLE_DUST_COLLECT_LOW 状态持续时间
uint32_t pole_dust_collect_complete_time = 0;     // POLE_DUST_COLLECT_COMPLETE 状态持续时间
uint32_t pole_test_mode_time = 0;                 // POLE_TEST_MODE 状态持续时间
uint32_t pole_test_mode_resync_time = 0;          // POLE_TEST_MODE_RESYNC 状态持续时间
uint32_t pole_test_mode_select_time = 0;          // POLE_TEST_MODE_SELECT 状态持续时间

/* 错误计数和统计变量 */
static uint8_t consecutive_level = 0;  // 连续电平计数

/* 队列满计数 */
uint16_t queue_if_full_flag; 




/* ========================== 改变状态、配置当前状态结构体、打印时间统计 ========================== */

/* 获取状态名称字符串 */
const char* get_state_name(pole_decode_state_e state)
{
    switch(state)
    {
        case POLE_RESYNC:                return "POLE_RESYNC";
        case POLE_HIGH:                  return "POLE_HIGH";
        case POLE_LOW:                   return "POLE_LOW";
        case POLE_DUST_COLLECT:          return "POLE_DUST_COLLECT";
        case POLE_DUST_COLLECT_HIGH:     return "POLE_DUST_COLLECT_HIGH";
        case POLE_DUST_COLLECT_LOW:      return "POLE_DUST_COLLECT_LOW";
        case POLE_DUST_COLLECT_COMPLETE: return "POLE_DUST_COLLECT_COMPLETE";
        case POLE_TEST_MODE:             return "POLE_TEST_MODE";
        case POLE_TEST_MODE_RESYNC:      return "POLE_TEST_MODE_RESYNC";
        case POLE_TEST_MODE_SELECT:      return "POLE_TEST_MODE_SELECT";
        default:                         return "UNKNOWN";
    }
}

/* 状态切换时打印时间统计 */
void print_state_time(pole_decode_state_e old_state, pole_decode_state_e new_state, uint16_t duration)
{
    
#if ENABLE_DEBUG_PRINT
    
    printf("[极片状态机] %s -> %s, 持续时间: %d (单位: 0.1ms)\r\n", 
               get_state_name(old_state), 
               get_state_name(new_state), 
               duration);
#endif
    
}

/* 状态切换辅助函数:达到切换状态的条件的时候调用，自动记录和打印时间 */
static void change_state(pole_decode_t *decode_p, pole_decode_state_e new_state)
{
    pole_decode_state_e old_state = decode_p->state;
    uint16_t duration = decode_p->timer;
    
    /* 如果状态真的改变了，保存旧状态的持续时间到对应的全局变量 */
    if(old_state != new_state && duration > 0)
    {       
        /* 打印时间统计 */
        print_state_time(old_state, new_state, duration);
    }
    
    /* 切换状态并重置计时器：新的状态、新的计时 */
    decode_p->state = new_state;
    decode_p->timer = 0;
}

/* 重置极片解码器：重置状态机到POLE_RESYNC，并将所有状态时间统计归零 */
void pole_decoder_reset(void)
{
    pole_decode_t *decode_p = &pole_decode;
    
    /* 重置状态机到POLE_RESYNC状态 */
    change_state(decode_p, POLE_RESYNC);
    
    /* 重置连续电平计数 */
    consecutive_level = 0;
    
    /* 将所有状态时间统计变量归零 */
    pole_resync_time = 0;
    pole_high_time = 0;
    pole_low_time = 0;
    pole_dust_collect_high_time = 0;
    pole_dust_collect_low_time = 0;
    pole_dust_collect_complete_time = 0;
    pole_test_mode_time = 0;
    pole_test_mode_resync_time = 0;
    pole_test_mode_select_time = 0;
}

void pole_detect_capture(void)
{
	uint8_t state;
	uint16_t adc_value;
	
	/* 确保队列已初始化 */
	if (pole_rx_queue_init_flag == 0)
	{
		queue_circular_init(&pole_rx_queue, pole_rx_buffer, POLE_QUEUE_LEN);
		pole_rx_queue_init_flag = 1;
	}
	
	adc_value = get_charging_cur();
	
	if(adc_value <= POLE_ADC_THRESHOLD)
	{
		state = 0;
	}
	else
	{
		state = 1;
	}
	
	pole_working_byte <<= 1;
	pole_working_byte |= state;
	
	pole_capture_counter++;
	if(pole_capture_counter >= 8)
	{
		if(!queue_circular_is_full(&pole_rx_queue))
		{
            queue_circular_put(&pole_rx_queue,pole_working_byte);
            pole_working_byte = 0;
            pole_capture_counter = 0;
		}
        else
        {
            queue_if_full_flag++;
        }
	}
}

uint32_t count_low = 0;
uint8_t pole_decoder_process(void)
{
	uint8_t bit_state_8;
	uint8_t state;
	pole_decode_t *decode_p = &pole_decode;
	
	/* 检查队列是否为空 */
	if(queue_circular_is_empty(&pole_rx_queue))
	{
		return 0;
	}
	
	/* 从队列获取一个字节 */
	bit_state_8 = queue_circular_get(&pole_rx_queue);
	
	/* 逐位处理 */
	for(uint8_t i = 0; i < 8; i++)
	{
		state = (bit_state_8 >> (7 - i)) & 0x01;
		decode_p->timer++;

		switch(decode_p->state)
		{
            /* =======正常模式、产测模式，默认状态都是持续高电平，卡在这一状态 ======= */
			case POLE_RESYNC:
			{
				if(state == 0)
				{
					// 检测到低电平，consecutive_level重置，继续等待连续的高电平
					consecutive_level = 0;
				}
				else
				{
					// 连续监测到高电平，计数加1
					consecutive_level++;
					if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
					{
                        pole_resync_time = decode_p->timer;
						// 连续5个高电平，进入POLE_HIGH状态
						change_state(decode_p, POLE_HIGH);
						consecutive_level = 0;
					}
					// 未达到高电平阈值数量，继续等待
				}
			}
			break;
            
            case POLE_HIGH:
            {
				if(state == 1)
				{
					// 检测到高电平，consecutive_level重置，继续等待连续的低电平
					consecutive_level = 0;
				}
				else
				{
					// 连续监测到低电平，计数加1
					consecutive_level++;
					if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
					{
						// 保存POLE_HIGH状态的持续时间
						pole_high_time = decode_p->timer;
						// 连续5个低电平，进入POLE_LOW状态
						change_state(decode_p, POLE_LOW);
						consecutive_level = 0;
					}
					// 未达到低电平阈值数量，继续等待
				}                
            }
            break;

            /* =======根据第一个低电平持续时间，判断进入集尘模式或是产测模式======= */
			case POLE_LOW:
			{
				/* 超时处理：如果在此状态持续超过800ms，返回到POLE_RESYNC状态 */
				if(decode_p->timer > 8000)
				{
					pole_decoder_reset();
				}
				// 在低电平状态下，持续计时
				else if(state == 0)
				{
                    consecutive_level = 0;
                    count_low++;
				}
				else
				{
                    consecutive_level++;           
					
                    /* 超时前接收到连续的高电平，分类讨论进行状态的变化 */
                    if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
                    {
						pole_low_time = decode_p->timer;
                        if(pole_low_time >= POLE_LOW_TIME_200MS_MIN && 
                            pole_low_time <= POLE_LOW_TIME_200MS_MAX)
                         {
                            /* 检测到集尘信号，更新极片通讯时间戳并切换回模式一 */
                            dust_mode_on_pole_comm();
                            change_state(decode_p, POLE_DUST_COLLECT_HIGH);
                         }
                         else if(pole_low_time >= POLE_LOW_TIME_400MS_MIN && 
                                 pole_low_time <= POLE_LOW_TIME_400MS_MAX)
                         {
                             /* 接收到进入产测模式指令，首先确保当下工作模式已经变为"产测模式" */
                            base_work_mode = BASE_WORK_MODE_TEST;

                            sent_buff[0]=0xaa;
                            sent_buff[1]=0x01;
                            sent_buff[2]=0x01;
                            sent_buff[3]=0x01;
                            sent_buff[4]=sent_buff[0]+sent_buff[1]+sent_buff[2]+sent_buff[3];		
                            set_ir_send_count(3);	
                            set_ir_sent_bite(5);

                            change_state(decode_p, POLE_TEST_MODE);

                            LED_OFF();

                         }
                    }
				}
			}
			break;

            /* =======集尘模式分支======= */
			case POLE_DUST_COLLECT_HIGH:
			{
				/* 超时处理：如果在此状态持续超过800ms，返回到POLE_RESYNC状态 */
				if(decode_p->timer > 8000)
				{
					pole_decoder_reset();
				}
                
                if(state == 1)
				{
					consecutive_level = 0;
				}
				else
				{
					// 连续监测到低电平，计数加1
					consecutive_level++;
					if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
					{
						// 保存POLE_DUST_COLLECT_HIGH状态的持续时间
						pole_dust_collect_high_time = decode_p->timer;
						change_state(decode_p, POLE_DUST_COLLECT_LOW);
						consecutive_level = 0;
					}
					// 否则继续等待
				}
			}
			break;

            case POLE_DUST_COLLECT_LOW:
            {
				/* 超时处理：如果在此状态持续超过800ms，返回到POLE_RESYNC状态 */
				if(decode_p->timer > 8000)
				{
					pole_decoder_reset();
				}
                
                if(state == 0)
				{
					consecutive_level = 0;
				}
				else
				{
					consecutive_level++;
					if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
					{
						// 保存POLE_DUST_COLLECT_LOW状态的持续时间
						pole_dust_collect_low_time = decode_p->timer;
						change_state(decode_p, POLE_DUST_COLLECT_COMPLETE);
						consecutive_level = 0;
					}
					// 否则继续等待
				} 
            }
            break;

            case POLE_DUST_COLLECT_COMPLETE:
            {
                if(dust_bag_state == DUST_BAG_STATE_UNSTALL)
				{
					sent_buff[0]=0x0b;
					sent_buff[1]=0x01;
					sent_buff[2]=0x0c;
				}
				else if(dust_bag_time >216000*1000)
				{
					sent_buff[0]=0x0b;
					sent_buff[1]=0x00;
					sent_buff[2]=0x0b;
				}
				else
				{
					sent_buff[0]=0x0e;
					sent_buff[1]=0x02;
					sent_buff[2]=0x10;
					need_duty = 1 ;	
				}	
				set_ir_sent_bite(3);
				set_ir_send_count(5);
                
                /* 保存POLE_DUST_COLLECT_COMPLETE状态的持续时间 */
				pole_dust_collect_complete_time = decode_p->timer;
                /* 完成集尘，回归正常模式的初始态 */
				pole_decoder_reset();
            }
            break;

            /* =======产测模式分支======= */
			case POLE_TEST_MODE:
			{
                if(state == 0)
				{
					consecutive_level = 0;
				}
				else
				{
					consecutive_level++;
					if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
					{
						// 保存POLE_TEST_MODE状态的持续时间
						pole_test_mode_time = decode_p->timer;
						change_state(decode_p, POLE_TEST_MODE_RESYNC);
						consecutive_level = 0;
					}
					// 否则继续等待
				}
			}
			break;

            case POLE_TEST_MODE_RESYNC:
            {
                if(state == 1)
				{
					consecutive_level = 0;
				}
				else
				{
					consecutive_level++;
					if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
					{
						// 保存POLE_TEST_MODE_RESYNC状态的持续时间
						pole_test_mode_resync_time = decode_p->timer;
						change_state(decode_p, POLE_TEST_MODE_SELECT);
						consecutive_level = 0;
					}
					// 否则继续等待
				}
            }
            break;

            case POLE_TEST_MODE_SELECT:
            {
				/* 超时处理：如果在此状态持续超过3000ms，返回到POLE_RESYNC状态 */
				if(decode_p->timer > 30000)
				{
					pole_decoder_reset();
				}
				// 在低电平状态下，持续计时
				if(state == 0)
				{
                    consecutive_level = 0;
				}
				else
				{
                    consecutive_level++;
					
                    if(consecutive_level >= CONSECUTIVE_LEVEL_COUNT)
                    {
						pole_test_mode_select_time = decode_p->timer;
						if(pole_test_mode_select_time >= POLE_LOW_TIME_600MS_MIN && pole_test_mode_select_time <= POLE_LOW_TIME_600MS_MAX)
						{
                            sent_buff[0]=0xaa;
                            sent_buff[1]=0x04;
                            sent_buff[2]=0x02;
                            sent_buff[3]=soft_version;
                            sent_buff[4]=version[0];
                            sent_buff[5]=version[1];
                            sent_buff[6]=ac_frequency;
                            sent_buff[7]=sent_buff[0]+sent_buff[1]+sent_buff[2]+sent_buff[3]+sent_buff[4]+sent_buff[5]+sent_buff[6];		
                            set_ir_send_count(3);		
                            set_ir_sent_bite(8);
						}
						else if(pole_test_mode_select_time >= POLE_LOW_TIME_800MS_MIN && pole_test_mode_select_time <= POLE_LOW_TIME_800MS_MAX)
						{
                            sent_buff[0]=0xaa;
                            sent_buff[1]=0x04;
                            sent_buff[2]=0x03;
                            sent_buff[3]=(0x02-dust_bag_state);
                            sent_buff[4]=get_charging_cur()>>8;
                            sent_buff[5]=get_charging_cur()&0xff;
                            sent_buff[6]=(0x02-get_gpio_level(CW_GPIOB,GPIO_PIN_1));
                            sent_buff[7]=sent_buff[0]+sent_buff[1]+sent_buff[2]+sent_buff[3]+sent_buff[4]+sent_buff[5]+sent_buff[6];		
                            set_ir_send_count(3);	
                            set_ir_sent_bite(8);
                            
                            set_led_twinkle_time(2000);
						}
						else if(pole_test_mode_select_time >= POLE_LOW_TIME_1000MS_MIN && pole_test_mode_select_time <= POLE_LOW_TIME_1000MS_MAX)
						{
                            if((timer_elapsed(dust_collect_time_cnt) > 5 * 1000))
                            {
                                sent_buff[0]=0xaa;
                                sent_buff[1]=0x01;
                                sent_buff[2]=0x04;
                                sent_buff[3]=0x01;
                                sent_buff[4]=sent_buff[0] +sent_buff[1]+sent_buff[2]+sent_buff[3];
                                /*使能红外发码*/
                                set_ir_send_count(3);
                                set_ir_sent_bite(5);

                                dust_absorption_time = 2;
                                need_duty = 1;
                            }	
						}
						else if(pole_test_mode_select_time >= POLE_LOW_TIME_1200MS_MIN && pole_test_mode_select_time <= POLE_LOW_TIME_1200MS_MAX)
						{
                            if((get_time_for_dust_finish() > 5 * 1000))
                            {
                                sent_buff[0]=0xaa;
                                sent_buff[1]=0x01;
                                sent_buff[2]=0x04;
                                sent_buff[3]=0x01;
                                sent_buff[4]=sent_buff[0] +sent_buff[1]+sent_buff[2]+sent_buff[3];
                                /*使能红外发码*/
                                set_ir_send_count(3);
                                set_ir_sent_bite(5);

                                dust_absorption_time = 2;
                                need_duty = 1;                                
                            }
						}
						else if(pole_test_mode_select_time >= POLE_LOW_TIME_2000MS_MIN && pole_test_mode_select_time <= POLE_LOW_TIME_2000MS_MAX)
						{
                            /* 退出产测，回归正常模式的初始态 */
							pole_decoder_reset();
                            base_work_mode = BASE_WORK_MODE_NORMAL;
						}                       
                    }
				}
            }
            break;


			default:
			{

			}
			break;
		}
	}
	
	return 1;  // 返回1表示处理了数据
}

