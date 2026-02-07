#include "func_ir_decoder.h"
#include "base_state.h"
#include <stdint.h>
#include "cw32l010_gpio.h"
#include "utils_queue.h"



#define IS_LEVEL_INVERT 1
static uint8_t ir_working_byte;
static uint8_t ir_capture_counter;

/* 队列长度定义 */
#define IR_QUEUE_LEN                64
/* 储存原始接收数据的队列 */
uint8_t ir_rx_buffer[IR_QUEUE_LEN] = {0};
static queue_circular_t ir_rx_queue;
static uint8_t ir_rx_queue_init_flag = 0;  // 队列初始化标志

void ir_detect_capture(void)
{
	/* 正常模式、机器人不在座，直接返回退出 */
	if(base_work_mode == BASE_WORK_MODE_NORMAL && robot_at_dock_state == ROBOT_STATE_NOT_AT_DOCK)
	{
		return;
	}
    
	/* 确保队列已初始化 */
	if (ir_rx_queue_init_flag == 0)
	{
		queue_circular_init(&ir_rx_queue, ir_rx_buffer, IR_QUEUE_LEN);
		ir_rx_queue_init_flag = 1;
	}
    
    uint8_t state;
    uint8_t index = 0;
    /* 红外接收灯：有光的时候为低电平，无光的时候为高电平；所以接收灯的电平结果转化为灯的亮灭结果需要取反 */	
	state = GPIO_ReadPin(CW_GPIOA ,GPIO_PIN_3 );
	state ^= IS_LEVEL_INVERT;				
	ir_working_byte <<= 1;
	ir_working_byte |= state;
	ir_capture_counter++;
	if (ir_capture_counter >= 8)
	{
		if(!queue_circular_is_full(&ir_rx_queue))
		{
			queue_circular_put(&ir_rx_queue,ir_working_byte);
			ir_working_byte = index;
			ir_capture_counter = 0;
		}
	}
}

