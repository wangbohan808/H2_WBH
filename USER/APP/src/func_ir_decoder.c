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


/* 储存解码后数据的队列 */
#define IR_DECODED_QUEUE_LEN        32      // 解码后数据队列长度
uint8_t ir_decoded_buffer[IR_DECODED_QUEUE_LEN] = {0};
queue_circular_t ir_decoded_queue = {ir_decoded_buffer, 0, 0, IR_DECODED_QUEUE_LEN};
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

/* 按协议接收到的原始数据包 */
uint8_t ir_rx_packet[32] = {0x0};

// 协议解析状态机枚举
typedef enum {
    PARSE_STATE_WAIT_SYNC_BYTE1 = 0,      // 等待同步字节1 (0x69)
    PARSE_STATE_WAIT_SYNC_BYTE2,          // 等待同步字节2 (0x96)
    PARSE_STATE_PROTOCOL_TYPE,            // 协议类型 (0x14)
    PARSE_STATE_PROTOCOL_VERSION,         // 协议版本
    PARSE_STATE_MESSAGE_SEQ,              // 消息序号
    PARSE_STATE_CMD_ID,                   // 命令ID (2字节)
    PARSE_STATE_MSG_LENGTH,                // 消息长度 (2字节)
    PARSE_STATE_HEADER_CHECKSUM,          // 消息头校验
    PARSE_STATE_BODY_CHECKSUM,            // 消息体校验和
    PARSE_STATE_DEVICE_TYPE,              // 设备类型
    PARSE_STATE_OTA_TYPE,                 // OTA类型/控制类型
    PARSE_STATE_RESERVED_FIELD,           // 保留字段
    PARSE_STATE_FIRMWARE_SIZE,            // 固件大小 (4字节)
    PARSE_STATE_PACKET_NUMBER,            // 数据包序号 (2字节)
    PARSE_STATE_DATA_PAYLOAD,             // 数据载荷 (16字节)
    PARSE_STATE_CTRL_CMD_BYTE2,           // 控制指令第二个字节
    PARSE_STATE_CTRL_CMD_BYTE3            // 控制指令第三个字节（校验）
}PROTOCOL_PARSE_STATE_E;
PROTOCOL_PARSE_STATE_E parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;

uint8_t recv_data = 0xff;
/* 记录接收到的字节在数组中的位置 */
uint8_t data_index = 0;

/* 两个字节的message_id用于路由不同消息到对应的处理逻辑 */
uint8_t recv_len = 0;
uint16_t message_id = 0;

uint8_t check_sum(uint8_t * arr ,uint8_t len)
{
    uint8_t sum = 0;
	for(int i = 0; i <len; i++)
	{
	    sum += arr[i] ;
	}
	return sum ;
}

uint8_t check_sum1 = 0;            //消息体校验和

uint32_t Firmware_size = 0;
uint8_t recvive_ok_flag = 0;

uint16_t Numbur = 0;

void ir_rx_packet_parse(uint8_t *packet, uint8_t len)
{
    while(queue_circular_is_empty(&ir_decoded_queue))
    {
        recv_data = queue_circular_get(&ir_decoded_queue);

        if(data_index >= 32)
        {
            data_index = 0;
        }

        switch(parse_state)
        {
            case PARSE_STATE_WAIT_SYNC_BYTE1:
            {
                if(recv_data == 0x69)
                {
                    data_index = 0;
                    /* 先进行储存，后进行数组的序号递增 */
                    packet[data_index++] = recv_data;
                    parse_state = PARSE_STATE_WAIT_SYNC_BYTE2;
                }
                else
                {
                    packet[data_index++] = recv_data;
                    parse_state = PARSE_STATE_CTRL_CMD_BYTE2;
                }
            }break;

            case PARSE_STATE_WAIT_SYNC_BYTE2:
            {
                if(recv_data == 0x96)
                {
                    packet[data_index++] = recv_data;
                    parse_state = PARSE_STATE_PROTOCOL_TYPE;
                }
                else
                {
                    data_index = 0;
                    parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;
                }
            }break;

            case PARSE_STATE_PROTOCOL_TYPE:
            {
                packet[data_index++] = recv_data;
                parse_state = PARSE_STATE_PROTOCOL_VERSION;
            }break;

            case PARSE_STATE_PROTOCOL_VERSION:
            {
                packet[data_index++] = recv_data;
                parse_state = PARSE_STATE_MESSAGE_SEQ;
            }break;

            case PARSE_STATE_MESSAGE_SEQ:
            {
                packet[data_index++] = recv_data;
                parse_state = PARSE_STATE_CMD_ID;
            }break;

            case PARSE_STATE_CMD_ID:
            {
                packet[data_index++] = recv_data;
                recv_len++;
                if(recv_len >= 2)
                {
                    recv_len = 0;
                    parse_state = PARSE_STATE_MSG_LENGTH;
                    message_id = packet[5] << 8 | packet[6];
                }
            }break;
            
            case PARSE_STATE_MSG_LENGTH:
            {
                packet[data_index++] = recv_data;
                recv_len++;
                if(recv_len >= 2)
                {
                    recv_len = 0;
                    parse_state = PARSE_STATE_HEADER_CHECKSUM;
                }
            }break;

            case PARSE_STATE_HEADER_CHECKSUM:
            {
                packet[data_index++] = recv_data;
                if(check_sum(packet, 9) == packet[9])
                {
                    parse_state = PARSE_STATE_BODY_CHECKSUM;
                }
                else
                {
                    data_index = 0;
                    parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;
                }
            }break;

            case PARSE_STATE_BODY_CHECKSUM:
            {
                packet[data_index++] = recv_data;
                check_sum1 = recv_data;
                parse_state = PARSE_STATE_DEVICE_TYPE;
            }break;

            case PARSE_STATE_DEVICE_TYPE:
            {
                packet[data_index++] = recv_data;
                parse_state = PARSE_STATE_OTA_TYPE;
            }break;

            case PARSE_STATE_OTA_TYPE:
            {
                packet[data_index++] = recv_data;
                if(recv_data == 0)
                {
                    parse_state = PARSE_STATE_RESERVED_FIELD;
                    ota_ok = 0;
                }
                else if(recv_data == 0x10)
                {
                    parse_state = PARSE_STATE_PACKET_NUMBER;
                    ota_ok = 0;          
                }
                else if(recv_data == 0x04)
                {
                    recvive_ok_flag = 1 ;               //升级成功
                    ota_ok = 1;
                    data_index = 0;
                    parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;
                }
                else
                {
                     data_index = 0;
                     parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;
                     ota_ok = 0;
                }
            }break;

            case PARSE_STATE_RESERVED_FIELD:
            {
                packet[data_index++] = recv_data;
                parse_state = PARSE_STATE_FIRMWARE_SIZE;
            }break;

            case PARSE_STATE_FIRMWARE_SIZE:
            {
                packet[data_index++] = recv_data;
                recv_len++;
                if(recv_len >= 4)
                {
                    if(check_sum(packet+11,7) == check_sum1)        //6
                    {
                         recvive_ok_flag = 1;                              //接收到固件大小
                         if(message_id == 0x16)
                         {
                             base_work_mode = BASE_WORK_MODE_IR_OTA;
                             Firmware_size = packet[14]<<24|packet[15]<<16|packet[16]<<8|packet[17] ;
                         }
                    }
                    recv_len = 0; 
                    data_index = 0;
                    parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;
               }
            }break;
            
            case PARSE_STATE_PACKET_NUMBER:
            {
                packet[data_index++] = recv_data;
                recv_len++;
                if(recv_len >= 2)
                {
                    recv_len = 0;
                    parse_state = PARSE_STATE_DATA_PAYLOAD;
                    Numbur = recv_data_buf[13];
                }
            }break;

            case PARSE_STATE_DATA_PAYLOAD:
            {
                packet[data_index++] = recv_data;
                recv_len++;
                if(recv_len >= 16)
                {
                    recv_len = 0;
                    parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;
                    data_index = 0;
                    if(check_sum(packet+11,20) == check_sum1)          //校验通过，写数据
                    {
                        recvive_ok_flag = 1;	                               //接收到数据
                    }
                }
            }break;

            case PARSE_STATE_CTRL_CMD_BYTE2:
            {
                packet[data_index++] = recv_data;
                parse_state = PARSE_STATE_CTRL_CMD_BYTE3;
            }break;

            case PARSE_STATE_CTRL_CMD_BYTE3:
            {
                packet[data_index++] = recv_data;
                data_index = 0;
                parse_state = PARSE_STATE_WAIT_SYNC_BYTE1;

                if(((packet[0]+packet[1])&0xff) ==packet[2])
                {
                    recvive_ok_flag = 1;
                }
            }return;
        }
    }
}


