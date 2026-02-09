#include "func_ir_decoder.h"
#include "base_state.h"
#include <stdint.h>
#include "cw32l010_gpio.h"
#include "utils_queue.h"



#define IS_LEVEL_INVERT 1
static uint8_t ir_working_byte;
static uint8_t ir_capture_counter;



/* 储存原始接收数据的队列 */
#define IR_QUEUE_LEN                64
uint8_t ir_rx_buffer[IR_QUEUE_LEN] = {0};
queue_circular_t ir_rx_queue = {ir_rx_buffer, 0, 0, IR_QUEUE_LEN};

/* 储存解码后数据的队列 */
#define IR_DECODED_QUEUE_LEN        32      // 解码后数据队列长度
uint8_t ir_decoded_buffer[IR_DECODED_QUEUE_LEN] = {0};
queue_circular_t ir_decoded_queue = {ir_decoded_buffer, 0, 0, IR_DECODED_QUEUE_LEN};

void ir_detect_capture(void)
{
	/* 正常模式、机器人不在座，直接返回退出 */
	if(base_work_mode == BASE_WORK_MODE_NORMAL && robot_at_dock_state == ROBOT_STATE_NOT_AT_DOCK)
	{
		return;
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

void ir_dock_resync_init(ir_decode_t * decode_p)
{
    decode_p->state = IR_RESYNC;
    decode_p->timer = 0;
	decode_p->count = 0; 
	decode_p->decode_value = 0;
	decode_p->bits_count = 0;
}

ir_decode_t ir_decode = {IR_RESYNC, 0};

void ir_decoder_process(void)
{
    uint8_t value = 0;
    uint8_t state = 0;
    ir_decode_t *decode_p = &ir_decode;
    queue_circular_t *decoded_queue_p = &ir_decoded_queue;
    queue_circular_t *ir_rx_queue_p = &ir_rx_queue;
    
    // 从输入队列中取出所有待处理的数据
    while (!queue_circular_is_empty(ir_rx_queue_p))
    {
        // 从队列中取出一个字节（8位采样数据）
        value = queue_circular_get(ir_rx_queue_p);
        
        // 从高位到低位逐位处理
        for (uint8_t i = 0; i < 8; i++)
        {
            // 提取当前位（从最高位开始）
            state = (value >> (7 - i)) & 0x01;
            
            // ========== 状态机处理（内联） ==========
            decode_p->timer++;  // 每次调用timer自增（100us为单位）
            
            switch (decode_p->state)
            {
				/* 如果一直保持低电平状态，状态机一直停留在同步态 */
                case IR_RESYNC:  // 等待信号进入同步
                    if (state == 0)
                    {
                        ir_dock_resync_init(decode_p);
                    }
                    /* 连续40个高电平，才会从同步态进入检测前导码的状态（推测扫地机发送了超过4ms的高电平） */
                    else if (decode_p->timer > DOCK_RESYNC_TICKS)  // 40个tick = 4ms
                    {
                        decode_p->state = IR_HEADER;
                    }
                    break;
                
				/* 只有上述接收一段高电平码值之后，才会进入前导码的判断 */
                case IR_HEADER:  // 检测前导码
                    if (state == 0)
                    {
                        decode_p->count++;  // 统计低电平时间
                        /* 接收非连续的25个低电平就可以进入下一个解码态（在这一状态停留没有超时逻辑，这一步似乎没有必要） */
                        if (decode_p->count >= DOCK_HEADER_LOW)  // 25个tick = 2.5ms
                        {
                            decode_p->state = IR_DATA_BIT_HIGH;
                            decode_p->timer = 0;
                            decode_p->count = 0;
                        }
                    }
                    break;
                
				/* 无论是0还是1，进行调制之后的第一位都是1 */
                case IR_DATA_BIT_HIGH:  // 数据位高电平
                    if (decode_p->count == 0 && state == 1)  // 首次检测到高电平
                    {
                        decode_p->timer = 1;
                    }
                    if (state == 1)
                    {
                        decode_p->count++;  // 统计高电平时间
                    }
                    /* 接收到一个高电平信号，等待0.8ms，直接进入下一个状态 */
                    if (decode_p->timer >= 9 - 1)  // 8个tick = 0.8ms
                    {
                        decode_p->state = IR_DATA_BIT_LOW;
                        decode_p->timer = 0;
                        decode_p->count = 0;
                    }
                    break;
                
				/* 首次检测到低电平之后开始计数，解码8位并再次接收到高电平进行判断 */
                /* 1->100  0->10，10个1在上一步解调完毕，根据0的个数判断结果是0还是1 */
                case IR_DATA_BIT_LOW:  // 数据位低电平（关键解码）
                    if (decode_p->count == 0 && state == 0)  // 首次检测到低电平
                    {
                        decode_p->timer = 1;
                    }
                    if (state == 0)
                    {
                        decode_p->count++;  // 统计低电平时间
                    }
 
                    /* 只有结束码才会进入这里 */
                    if ((decode_p->timer >= 8) && (decode_p->count < 1))
                    {
                        // 解码错误，重置状态机
                        ir_dock_resync_init(decode_p);
                        decode_p->decode_value = 0;
                        break;  // 退出switch，继续处理下一个位
                    }
                    
                    // 正常解码：检测到下一个高电平，一位解码完成
                    if (decode_p->timer >= 8 && state == 1)
                    {
                        decode_p->decode_value <<= 1;  // 左移1位
                        
                        // 根据低电平持续时间判断0/1
                        // 数据位1：低电平约40点(4ms) >= 12点 → 解码为1
                        // 数据位0：低电平约10点(1ms) < 12点 → 解码为0
                        if (decode_p->count >= DOCK_DATA_BIT_VALUE_1_TICK)  // >= 12个tick = 1.2ms
                        {
                            decode_p->decode_value++;  // 最低位置1
                        }
                        
                        decode_p->bits_count++;  // 位计数+1
                        decode_p->state = IR_DATA_BIT_HIGH;  // 准备解码下一位
                        decode_p->count = 0;
                        decode_p->timer = 0;
                        
                        // 8位解码完成，组成1字节
                        if (decode_p->bits_count >= 8)
                        {
                            queue_circular_put(decoded_queue_p, (decode_p->decode_value & 0x0FF));
                            decode_p->bits_count = 0;
                            decode_p->decode_value = 0;
                        }
                    }
                    break;
            }
            // ========== 状态机处理结束 ==========
        }
    }
    
}

