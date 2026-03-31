#include "func_ir_decoder.h"
#include "base_state.h"
#include <stdint.h>
#include "cw32l010_gpio.h"
#include "utils_queue.h"
#include "tim_cfg.h"
#include "func_ir_ack.h"
#include "base_event.h"
#include "func_dust_collect.h"
#include "dev_led.h"
#include "cw32l010_flash.h"
#include "hal_adc.h"

/* =========================== 原始红外数据解码为原始字节 ========================= */

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

/* ========================= 原始字节处理为协议帧 ========================= */

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

/* 不断、临时接收一字节解码后的原始数据，从而完成数据帧的封装 */
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
/* 暂存接收到的“消息体校验和（body checksum）”字节，随后在收完消息体后与本地计算的 check_sum(recv_data_buf+11, …) 对比，以判定该帧数据是否校验通过 */
uint8_t check_sum1 = 0;
/* 本次 OTA 升级固件（bin 镜像）的总字节数，用于控制分包写入 Flash 的边界处理（最后不足一包的补齐）以及最终对整段固件做 CRC 校验 */
uint32_t Firmware_size = 0;
/* 标记“已成功接收到并校验通过的一帧数据/事件” */
uint8_t receive_ok_flag = 0;
/* 记录当前接收到的 OTA 数据包序号（包编号），在 OTA_Process() 中与 last_number 对比做顺序校验/防乱序与防重包，只有满足连续递增时才写入 Flash*/
uint16_t Numbur = 0;
/* 0x04 = OTA 完成/成功指示（结束帧），触发基站进入 OTA 结束确认与升级切换流程（CRC 校验、写标志、复位：ota_ok = 1 */
uint8_t ota_ok = 0;
/* 记录进入OTA模式的时间，用于后续的超时判断 */
uint32_t timeout = 0 ;

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
                    receive_ok_flag = 1 ;               //升级成功
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
                         receive_ok_flag = 1;                              //接收到固件大小
                         if(message_id == 0x16)
                         {
                             timeout = timer_ms();
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
                    Numbur = packet[13];
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

                    /* 校验通过,标志一包数据接收完成 */
                    if(check_sum(packet+11,20) == check_sum1)         
                    {
                        receive_ok_flag = 1;	                               
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

                /* 校验通过,标志一包数据接收完成 */
                if(((packet[0]+packet[1])&0xff) ==packet[2])
                {
                    receive_ok_flag = 1;
                }
            }return;
        }
    }
}

/*
==================== 非OTA常规命令数据帧格式（board_data_info = 15/16 分支）====================

触发条件：
- 接收的第1字节 != 0x69 时，认为不是OTA协议帧，转入常规命令解析：
  recv_data_buf[data_index++] = first_byte;
  board_data_info = 15;

帧结构（长度固定为 3 字节）：
Byte0 : CMD0
Byte1 : CMD1
Byte2 : CHK  (校验)

校验规则（8-bit）：
CHK == (CMD0 + CMD1) & 0xFF

解析完成动作：
- 读满 3 字节后，若校验通过则 recvive_ok_flag = 1
- 示例特殊处理：
  若 CMD0 == 0x0E 且 CMD1 == 0x00，则：
    set_light_twinkle_time(0);
    set_base_work_mode(WORK_MODE_IDEL);

备注：
- 常规命令帧没有 0x69 0x96 这样的帧头/协议类型/长度字段，就是简单的 3 字节命令 + 累加校验。
*/


/*
==================== OTA协议数据帧格式（帧头 0x69 0x96）====================

通用头部（固定 11 字节，先做头校验）：
Byte0  : 0x69          // SOF0
Byte1  : 0x96          // SOF1
Byte2  : proto_type    // 发送侧写死 0x14
Byte3  : proto_ver
Byte4  : Number        // 消息序号
Byte5  : cmd_id_hi      \
Byte6  : cmd_id_lo       > message_id = (buf[5]<<8) | buf[6]   (大端)
Byte7  : len_hi
Byte8  : len_lo
Byte9  : hdr_sum        // 头校验：hdr_sum == sum(buf[0..8]) (mod 256)
Byte10 : body_sum       // 体校验：后续消息体的累加和（见不同命令）

累加和函数（8-bit）：
sum = Σ bytes (mod 256)    // 对应代码 check_sum()

-------------------- OTA命令 0x16：下发固件大小（进入OTA模式）--------------------
消息体（从 Byte11 开始）：
Byte11 : device_type
Byte12 : ota_type          // 必须为 0x00 才走“固件大小/16字节分包”流程
Byte13 : (reserved/unused) // 接收侧读取1字节，但只参与校验
Byte14 : fw_size_b3        \
Byte15 : fw_size_b2         \
Byte16 : fw_size_b1          > Firmware_size = b3<<24 | b2<<16 | b1<<8 | b0  (大端)
Byte17 : fw_size_b0         /

体校验覆盖范围（7字节）：
body_sum == sum(buf[11..17]) (mod 256)

-------------------- OTA命令 0x17：数据分包（每包16字节payload）--------------------
消息体（从 Byte11 开始）：
Byte11 : device_type
Byte12 : ota_type          // 通常 0x00
Byte13 : pkt_no            // 包序号 Numbur
Byte14 : pkt_no_inv        // 补码校验：pkt_no + pkt_no_inv == 0xFF
Byte15..Byte30 : payload[16]   // 固定16字节

体校验覆盖范围（20字节）：
body_sum == sum(buf[11..30]) (mod 256)

写Flash取数方式（payload 内 32-bit 小端）：
word0 = payload[0] | payload[1]<<8 | payload[2]<<16 | payload[3]<<24
... 共4个word = 16字节
*/


/*
 * ===================== IR OTA 流程（按当前工程代码梳理）=====================
 *
 * message_id 来源：
 * - 在 main_task.c 的接收解析里，从协议帧 recv_data_buf[5..6] 提取（大端）：
 *     message_id = (recv_data_buf[5] << 8) | recv_data_buf[6];
 *
 * 参与文件/函数：
 * - main_task.c : test_data_process()        // 负责收包、校验、提取 message_id / Firmware_size 等
 * - task_base_mode_process.c : task_base_mode_process()
 *                                       // 负责状态机：进入 OTA、擦除、ACK、CRC 校验、写标志、复位
 * - ir_ota.c : OTA_Process()               // 负责按包序号写 Flash、最后一包置 recv_ota_data_ok
 *
 * --------------------- 阶段 0：会话准备/查询版本（可选）----------------------
 * message_id = 0x18
 * - 设备动作：
 *   - 回版本信息（回包里填 version[1]）
 *   - 同时清 OTA 相关计数/状态（便于开始新一次升级会话）：
 *       page = 0;
 *       timeout = 0;
 *       last_number = 0;
 *       count_data = 0;
 *
 * --------------------- 阶段 1：下发整包 bin 的 CRC32 ------------------------
 * message_id = 0x24
 * - 设备动作：
 *   - Station_send_ack(0, 0x24);
 *   - 保存期望 CRC32（用于最终校验）：
 *       bin_crc_data = recv_data_buf[14..17] (拼 32bit)
 *
 * --------------------- 阶段 2：启动 OTA（固件大小）→ 进入 OTA 模式 ----------
 * message_id = 0x16
 * - 发生点 A（main_task.c 收包解析）：
 *   - 当解析到“固件大小”字段且消息体校验通过时：
 *       Firmware_size = recv_data_buf[14..17] (拼 32bit)
 *   - 若 message_id == 0x16：
 *       set_base_work_mode(WORK_MODE_IR_OTA);
 *       timeout = timer_ms();   // 记录 OTA 超时起点
 *
 * - 发生点 B（task_base_mode_process.c 的 WORK_MODE_IR_OTA 状态机）：
 *   - 若 message_id == 0x16：
 *       1) 擦除升级区 Flash（循环擦页）
 *       2) Station_send_ack(0, 0x16);
 *       3) recv_ota_data_ok = 0; 等待后续数据包
 *
 * --------------------- 阶段 3：分包传输数据并写入 Flash（循环多次）----------
 * message_id = 0x17
 * - 在 WORK_MODE_IR_OTA 下：
 *   - 收到 0x17：调用 OTA_Process(recv_data_buf, ...) 写入 Flash
 *   - 随后回包：
 *       Station_send_ack(6, 0x17);
 *
 * - OTA_Process() 关键逻辑（ir_ota.c）：
 *   - 校验包序号 Numbur 与补码（recv_buff[13] + recv_buff[14] == 0xFF）
 *   - 按 16 字节(RECV_LEN=16)写入 Flash
 *   - 当处理到“最后不足 16 字节的一包”时，会：
 *       recv_ota_data_ok = 1;   // 表示数据接收/写入完成，可以进入校验阶段
 *
 * --------------------- 阶段 4：完成通知（注意：不是 message_id）-------------
 * - 在 main_task.c 解析中，如果某字段值 recv_data == 0x04（注释：升级成功）：
 *     recvive_ok_flag = 1;
 *     ota_ok = 1;
 * - 说明：这里的“完成/成功”触发不是靠 message_id，而是协议内某个“类型/状态字段”为 0x04。
 *
 * --------------------- 阶段 5：整包 CRC 校验 → 写标志 → 复位重启 ------------
 * - 触发条件：recv_ota_data_ok == 1 后，在 task_base_mode_process.c 中按 ota_ok 状态流转：
 *
 *   ota_ok == 1：
 *     - 等待红外发送忙闲信号切换（get_ir_enable_flag() == 1）→ ota_ok = 2
 *
 *   ota_ok == 2：
 *     - 当 get_ir_enable_flag() == 0 时进行整包 CRC 校验：
 *         if (crc32_update(0, start_add, Firmware_size) == bin_crc_data)  // 正确
 *             ota_ok = 3;
 *         else  // 错误
 *             ota_ok = 0;
 *             set_base_work_mode(WORK_MODE_IDEL);
 *             bin_crc_data = 0;
 *
 *   ota_ok == 3：
 *     - 写升级标志：
 *         app_flag_write(0xffffffff, app_update_flag_addr);
 *         app_flag_write1(0x87654321, ir_ota_addr);
 *     - 复位：
 *         NVIC_SystemReset();
 *
 * --------------------- message_id 与阶段对照 -------------------------------
 * - 0x18：阶段0（查询版本/会话准备，清计数）
 * - 0x24：阶段1（下发整包 CRC32）
 * - 0x16：阶段2（启动 OTA：固件大小→进入 OTA；OTA 模式内擦除 Flash + ACK）
 * - 0x17：阶段3（分包数据传输 + 写 Flash + ACK）
 *
 * 备注：
 * - 超时保护：WORK_MODE_IR_OTA 下若 50s 无有效接收则退出 OTA 回到空闲并清 bin_crc_data；
 *   另外 timeout 超 1 小时也会退出 OTA。
 * ==========================================================================
 */

 /* =========================== 协议帧处理为基础指令或者OTA数据储存 ========================= */
 
 void Station_send_ack(uint8_t ack,uint8_t cmd1)
{
    sent_buff[0] = 0x69;
    sent_buff[1] = 0x96;
    sent_buff[2] = 0x14;
    sent_buff[8] = 0x03;           //长度
    sent_buff[6] = cmd1;
    sent_buff[9] = check_sum(sent_buff, 9);
    sent_buff[12] = ack;
    sent_buff[10] = check_sum(sent_buff + 11, 2);
    set_ir_sent_bite(13);
    
    /* 0x04 = OTA 完成/成功指示（结束帧），触发基站进入 OTA 结束确认与升级切换流程（CRC 校验、写标志、复位：ota_ok = 1 */
    if (ota_ok == 1)
    {
        set_ir_send_count(6);
    }
    else
    {
        set_ir_send_count(2);
    }
}

static void handle_ir_normal_message(void);
static void handle_ir_version_request(void);
static void handle_ir_crc_info(void);

static void handle_work_mode_normal(void);
static void handle_work_mode_test(void);
static void handle_work_mode_ir_ota(void);

uint32_t timer_cnt = 0;
/* 用于确保“收到一帧有效数据后，延时一段时间再发送响应/处理”这段逻辑只执行一次，避免在主循环里反复进入发送 */
uint8_t sent_ok = 1;

void base_ir_mode_process(void)
{
    ir_rx_packet_parse(ir_rx_packet,sizeof(ir_rx_packet));

    /* 非OTA模式的常规命令、OTA模式下的两个特殊指令，需要优先处理 */
    if (base_work_mode != BASE_WORK_MODE_IR_OTA || message_id == 0x24 || message_id == 0x18)
    {
        if (receive_ok_flag == 1)
        {
            receive_ok_flag = 0;
            timer_cnt = timer_ms();
            sent_ok = 0;
        }
        if (timer_elapsed(timer_cnt) >= 60 && sent_ok == 0)
        {
            sent_ok = 1;

            if (message_id == 0)
            {
                handle_ir_normal_message();
            }
            else if (message_id == 0x18)
            {
                handle_ir_version_request();
            }
            else if (message_id == 0x24)
            {
                handle_ir_crc_info();
            }
            message_id = 0;
        }
    }
    
    /* 即使没有红外信息，根据当下的工作模式，也必须执行对应的工作流程 */
    switch(base_work_mode)
    {
        case BASE_WORK_MODE_NORMAL:
        {
            handle_work_mode_normal();
        }break;


        case BASE_WORK_MODE_TEST:
        {
            handle_work_mode_test();
        }break;


        case BASE_WORK_MODE_IR_OTA:
        {
            handle_work_mode_ir_ota();
        }
    }
}


/* =========================== 协议帧处理为基础指令或者特殊OTA指令预处理 ========================= */

/*** 普通红外命令：解析并创建对应事件 ***/
static void handle_ir_normal_message(void)
{
    uint32_t temp = 0;
    uint16_t cmd = 0;

    cmd = (uint16_t)((ir_rx_packet[0] << 8) | ir_rx_packet[1]);
    temp = ir_check_is_cmd(cmd);
    if (temp != 0)
    {
        base_event |= (1 << temp);
    }
}

/*** 版本请求消息处理（0x18） ***/

/* 记录上一次已成功写入的 OTA 包序号，用于与当前 Numbur 比较做连续性校验（防重包/乱序）*/
uint16_t last_number = 0;
/* 记录当前 Flash 页内已写入的 16B 数据块(一个数据帧大小)计数，用于计算写入偏移与判断页内块数是否满（满了就翻页） */
uint16_t count_data = 0;
/* 记录当前写入到 Flash 的页号/页索引，与 count_data 一起定位写入地址（start_add + page*512 + count_data*16） */
uint8_t page =0;

static void handle_ir_version_request(void)
{
    sent_buff[0] = 0x69;
    sent_buff[1] = 0x96;
    sent_buff[2] = 0x14;
    sent_buff[8] = 0x03;
    sent_buff[6] = message_id;
    sent_buff[9] = check_sum(sent_buff, 9);
    sent_buff[12] = version[1];
    sent_buff[10] = check_sum(sent_buff + 11, 2);

    set_ir_send_count(5);
    set_ir_sent_bite(13);

    page = 0;
    timeout = 0;
    last_number = 0;
    count_data = 0;
}

/*** CRC 信息消息处理（0x24） ***/
uint32_t bin_crc_data = 0;
static void handle_ir_crc_info(void)
{
    Station_send_ack(0, message_id);
    bin_crc_data = (uint32_t)ir_rx_packet[14] << 24 |
                   (uint32_t)ir_rx_packet[15] << 16 |
                   (uint32_t)ir_rx_packet[16] << 8  |
                   (uint32_t)ir_rx_packet[17];
}
 


/*=========================== 根据三种模式，主程序不断循环对应的流程 ===========================*/

 /* 进入老化模式步骤 */
typedef enum          
{
    WORK_MODE_FULL_GO_IDEL     = 0,
    WORK_MODE_FULL_GO_FIRST    = 1,     //步骤1
    WORK_MODE_FULL_GO_SECOND   = 2,     //步骤2
    WORK_MODE_FULL_GO_END      = 3,     //步骤3
}WORK_MODE_FULL_GO_STEP;
WORK_MODE_FULL_GO_STEP FULL_GO_STEP = WORK_MODE_FULL_GO_IDEL;

/* 老化模式测试项目 */
typedef enum           
{
    TEST_IDEL            = 0,
	TEST_IR              = 1,
    TEST_VACUUN_TEST     = 2,
    TEST_DUSTBUG_TEST    = 3,     
    TEST_LED_ON_TEST     = 4,     
	TEST_CURRENT_        = 5,
    TEST_END             = 6,	
}TEST_FULL_GO;
TEST_FULL_GO TEST_FULL_GO_SEC = TEST_IDEL ;

/* 产测模式下TEST_IR 项目的两步状态机（0: 记录起始时间并进入发射；1: 持续发红外引导码，超时约 1s 后停止并回发测试结果、退出该测试项）*/
uint8_t ir_test_step = 0;
/* 正常工作模式处理（含进入产测握手机制） */
static void handle_work_mode_normal(void)
{
    /* 非空闲模式优先；只有确保是空闲模式才会进入空闲模式的处理流程 */
    if (get_base_event(BASE_EVENT_TEST_START))
    {
        /* 进入产测模式的状态机标志位 */
        FULL_GO_STEP = WORK_MODE_FULL_GO_FIRST;

        /* 发送产测模式的ACK指令 */
        sent_buff[0] = 0xa6;
        sent_buff[1] = soft_version;
        sent_buff[2] = 0xa6 + sent_buff[1];
        set_ir_send_count(3);
        set_ir_sent_bite(3);

        /* 控制事件的全局变量 */
        base_event = 0;
    }

    /* 正常模式进入产测模式，需要三步握手的流程 */
    if (FULL_GO_STEP != WORK_MODE_FULL_GO_IDEL)
    {
        static uint32_t time_cnt = 0;
        switch (FULL_GO_STEP)
        {
            case WORK_MODE_FULL_GO_FIRST:
            {
                if (base_work_mode == BASE_WORK_MODE_NORMAL && robot_at_dock_state == ROBOT_STATE_NOT_AT_DOCK)
                {
                    time_cnt = timer_ms();
                }
                else
                {
                    if (get_base_event(BASE_EVENT_TEST_START1))
                    {
                        FULL_GO_STEP = WORK_MODE_FULL_GO_SECOND;
                        time_cnt = timer_ms();
                        /* 把发码模式配置成通讯模式 */
                        sent_buff[0] = 0x99;
                        sent_buff[1] = version[0];
                        sent_buff[2] = sent_buff[0] + sent_buff[1];
                        set_ir_sent_bite(3);
                        set_ir_send_count(3);
                    }
                    if (timer_elapsed(time_cnt) > 10 * 1000)           //超时
                    {
                        FULL_GO_STEP = WORK_MODE_FULL_GO_IDEL;
                    }
                }
            }break;

            case WORK_MODE_FULL_GO_SECOND:
            {
                if (base_work_mode == BASE_WORK_MODE_NORMAL && robot_at_dock_state == ROBOT_STATE_NOT_AT_DOCK)
                {
                    time_cnt = timer_ms();
                }
                else
                {
                    if (get_base_event(BASE_EVENT_TEST_START2))
                    {
                        FULL_GO_STEP = WORK_MODE_FULL_GO_END;
                        time_cnt = timer_ms();
                        /* 把发码模式配置成通讯模式 */
                        sent_buff[0] = 0x77;
                        sent_buff[1] = version[1];
                        sent_buff[2] = sent_buff[0] + sent_buff[1];
                        set_ir_send_count(3);
                        set_ir_sent_bite(3);
                    }
                    if (timer_elapsed(time_cnt) > 10 * 1000)
                    {
                        FULL_GO_STEP = WORK_MODE_FULL_GO_IDEL;
                    }
                }
            }break;

            case WORK_MODE_FULL_GO_END:
            {
                if (robot_at_dock_state == ROBOT_STATE_AT_DOCK)
                {
                    FULL_GO_STEP = WORK_MODE_FULL_GO_IDEL;
                    base_work_mode = BASE_WORK_MODE_TEST;
                    TEST_FULL_GO_SEC = TEST_IDEL;
                    set_led_twinkle_time(0);
                    LED_OFF();
                    TEST_FULL_GO_SEC = TEST_IR;
                    ir_test_step = 0;
                }
            }break;

            default:
                break;
        }
    }
    else
    {
        /* 确保不是正在转换为产测模式的过程，正式处理空闲模式处理流程 */
		if(get_base_event(BASE_EVENT_VACUUM_RM_STAR))
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
			
		}
		else if (get_base_event(BASE_EVENT_CHARGE_ON))
		{
			sent_buff[0]=0xba;
			sent_buff[1]=0x01;
			sent_buff[2]=0xbb;
			
			set_ir_sent_bite(3);
			set_ir_send_count(8);
		}
    }
}


/* 产测模式处理 */
uint8_t test_vacuun = 0;
DUST_BAG_STATE_E dustbug_state = DUST_BAG_STATE_UNSTALL;

static void handle_test_vacuum_event(void);
static void handle_test_dustbag_event(void);
static void handle_test_led_event(void);
static void handle_test_end_event(void);
static void handle_test_current_event(void);
static void handle_test_ir_error_event(void);
static void handle_test_state_machine(void);
static void base_mode_full_go_process(void);

/* 产测工作模式处理入口 */
static void handle_work_mode_test(void)
{
    set_ir_sent_bite(3);
    base_mode_full_go_process();
}


static void base_mode_full_go_process(void)
{
    handle_test_vacuum_event();
    handle_test_dustbag_event();
    handle_test_led_event();
    handle_test_end_event();
    handle_test_current_event();
    handle_test_ir_error_event();

    /* 获取事件，上述进行对应事件的初步处理后，接下来进行对应事件的进一步执行 */
    handle_test_state_machine();
}

/* 风机测试事件（含整机风机测试） */
static void handle_test_vacuum_event(void)
{
    /* 单次风机测试 */
    if (get_base_event(BASE_EVENT_TEST_VACUUN_START) == 1)
    {
        dust_absorption_time = 2;
        test_vacuun = 1;
    }

    /* 整机风机测试 */
    if (get_base_event(BASE_EVENT_TEST_VACUUN_START15) == 1)
    {
        dust_absorption_time = 8;
        test_vacuun = 1;
    }

    if (test_vacuun == 1)
    {
        test_vacuun = 0;
        LED_OFF();
        set_led_twinkle_time(0);
        if (get_time_for_dust_finish() > 5 * 1000)
        {
            sent_buff[0] = 0x66;
            sent_buff[1] = ac_frequency;
            sent_buff[2] = 0x66 + sent_buff[1];
            /* 使能红外发码 */
            set_ir_send_count(3);
            TEST_FULL_GO_SEC = TEST_VACUUN_TEST;
        }
    }
}

/* 尘袋测试事件 */
static void handle_test_dustbag_event(void)
{
    if (get_base_event(BASE_EVENT_TEST_DUSTBUG) == 1)
    {
        dustbug_state = dust_bag_state;
        sent_buff[0] = 0x67;
        sent_buff[1] = 0x02 - dustbug_state;
        sent_buff[2] = 0x67 + sent_buff[1];
        /* 使能红外发码 */
        set_ir_send_count(3);
        TEST_FULL_GO_SEC = TEST_DUSTBUG_TEST;
    }
}

/* 闪灯测试事件 */
static void handle_test_led_event(void)
{
    if (get_base_event(BASE_EVENT_TEST_LED_ON) == 1)
    {
        set_led_twinkle_time(1 * 1000);
        sent_buff[0] = 0x68;
        sent_buff[1] = 0x02;
        sent_buff[2] = 0x6a;
        /* 使能红外发码 */
        set_ir_send_count(3);
        TEST_FULL_GO_SEC = TEST_LED_ON_TEST;
    }
}

/* 测试结束事件 */
static void handle_test_end_event(void)
{
    if (get_base_event(BASE_EVENT_TEST_END) == 1)
    {
        sent_buff[0] = 0xcd;
        sent_buff[1] = 0xdc;
        sent_buff[2] = 0xa9;
        set_ir_send_count(3);
        base_work_mode = BASE_WORK_MODE_NORMAL;
        TEST_FULL_GO_SEC = TEST_IDEL;
        base_event = 0;
    }
}

/* 电流测试事件 */
static void handle_test_current_event(void)
{
    if (get_base_event(BASE_EVENT_TEST_CURRENT) == 1)
    {
        sent_buff[0] = (uint8_t)((0xd << 4) + (get_charging_cur() >> 8));
        sent_buff[1] = (uint8_t)get_charging_cur();
        sent_buff[2] = sent_buff[0] + sent_buff[1];
        set_ir_send_count(3);
    }
}

/* 红外错误测试事件 */
static void handle_test_ir_error_event(void)
{
    if (get_base_event(BASE_EVENT_TEST_IR_ERR) == 1)
    {
        TEST_FULL_GO_SEC = TEST_IR;
        ir_test_step = 0;
    }
}

/* 产测测试状态机执行 */
uint32_t ir_sent_time = 0;
static void handle_test_state_machine(void)
{
    switch (TEST_FULL_GO_SEC)
    {
        case TEST_IR:
        {
            switch (ir_test_step)
            {
                case 0:
                {
                    ir_sent_time = timer_ms();
                    ir_test_step = 1;
                }break;

                case 1:
                {
                    if (timer_elapsed(ir_sent_time) > 10 * 100)
                    {
                        ir_test_step = 0;
                        sent_buff[0] = 0x64;
                        sent_buff[1] = 0x00;
                        sent_buff[2] = 0x64;
                        set_ir_send_count(3);
                        TEST_FULL_GO_SEC = TEST_IDEL;
                    }
                }break;

                default:
                    break;
            }
        }break;

        case TEST_VACUUN_TEST:
        {
            if (robot_at_dock_state == ROBOT_STATE_AT_DOCK)
            {
                need_duty = 1;
                TEST_FULL_GO_SEC = TEST_IDEL;
            }
        }break;

        case TEST_DUSTBUG_TEST:
        {
            if (dustbug_state != dust_bag_state)
            {
                dustbug_state = dust_bag_state;
                set_ir_send_count(3);
                sent_buff[0] = 0x67;
                sent_buff[1] = 0x02 - dustbug_state;
                sent_buff[2] = 0x67 + sent_buff[1];
                TEST_FULL_GO_SEC = TEST_IDEL;
            }
        }break;

        case TEST_LED_ON_TEST:
        {
            if (led_twinkle_time == 0)
            {
                set_led_twinkle_time(1000);
            }
        }break;

        default:
            break;
    }
}



/* OTA 模式处理 */

/* 临时变量：暂存本次 app_flash_write() 要写入的 Flash 起始地址（Flash_address），随后作为 FLASH_WriteWords(start_add1, data, count) 的目标地址来执行实际写入*/
uint32_t start_add1;
/* 在 OTA 过程中把接收到的固件数据块按指定地址写入 Flash（解锁→写入多字→上锁） */
int32_t app_flash_write(uint32_t *data ,uint32_t Flash_address,uint16_t count)
{
    start_add1 = Flash_address;
    FLASH_UnlockAllPages();
    FLASH_WriteWords(start_add1,data,count);
    FLASH_LockAllPages();
    return 0;
}


/* 写“升级标志/状态”到指定 Flash 地址，写入前先擦除该地址所在页以保证可写入 */
int32_t app_flag_write(uint32_t data ,uint32_t start_add2)
{
    uint8_t page = 0;
    FLASH_UnlockAllPages();
    page = start_add2/512;
    FLASH_ErasePage(page);		
    FLASH_WriteWords(start_add2,&data,1);
    FLASH_LockAllPages();
        
    return 0;
}

/* 在指定 Flash 地址直接写一个标志字（不擦页），通常配合 app_flag_write() 写同页内的另一个标志位/参数 */
int32_t app_flag_write1(uint32_t data ,uint32_t start_add2)
{
    FLASH_UnlockAllPages();
    FLASH_WriteWords(start_add2,&data,1);
    FLASH_LockAllPages();
    
    return 0;
}

/* 定义 OTA 每个数据包有效载荷的固定长度为 16 字节，用于分包接收与计算写入偏移 */
#define RECV_LEN      16
/* 定义每个 Flash 页内可写入的 16 字节块数量为 32（$32 \times 16 = 512$），用于页内计数与翻页 */
#define PAGE_COUNT    32
/* 记录最后一个数据块不足 16 字节时需要补齐的字节数（代码里用 0xFF 填充） */
uint8_t last_len = 0;
/* 标记固件数据已接收并写入到最后一块（接收完成），用于触发后续 CRC 校验/写标志/复位流程 */
uint8_t recv_ota_data_ok = 0;
/* 作为写 Flash 的 4 个 32-bit 缓冲，把 16 字节数据打包成 4 个 word 以便 FLASH_WriteWords() 写入 */
uint32_t data_buff[4] = {0};
/* OTA 固件在 Flash 中的写入起始地址（基地址） */
#define start_add 0x0007000
/* 定义 Flash 页大小为 512 字节，用于地址计算与擦页/翻页逻辑 */
#define FLASH_PAGE_SIZE 512


/* 对 OTA 分包数据做序号连续性校验，将 16B 负载打包写入 Flash，并在写入最后一包时置位 recv_ota_data_ok 以触发后续收尾流程 */
void OTA_Process(uint8_t* recv_buff,uint8_t len )
{
	if(recv_buff[13]+recv_buff[14] ==0xff)    
	{
		if(Numbur == last_number+1)
		{
			if(Firmware_size-(page*512+(count_data)*RECV_LEN)>RECV_LEN)
			{
				for(int i = 0; i<4 ;i++)
				{
				   data_buff[i] = recv_buff[15+i*4] |recv_buff[16+i*4]<<8 |recv_buff[17+i*4] <<16 |recv_buff[18+i*4]<<24;
				}
				app_flash_write(data_buff,start_add+page*FLASH_PAGE_SIZE+count_data*RECV_LEN,4);
		  }
			else            
			{               //          12152               11776                   23*16  12144
				last_len = RECV_LEN-(Firmware_size-(page*512+count_data*RECV_LEN)) ;
				for(uint8_t i = 0;i<last_len;i++)
				{
				     recv_buff[15+(Firmware_size-(page*512+count_data*RECV_LEN))+i] = 0xff;
				}
				for(int i = 0; i<4 ;i++)
				{
				   data_buff[i] = recv_buff[15+i*4] |recv_buff[16+i*4]<<8 |recv_buff[17+i*4] <<16 |recv_buff[18+i*4]<<24;
				}
				app_flash_write(data_buff,start_add+page*FLASH_PAGE_SIZE+count_data*RECV_LEN,4);
				recv_ota_data_ok = 1;
			}
			last_number = Numbur;
			if(last_number == 255)
			{
			    last_number = 0;
			}
			count_data ++;
			if(count_data>PAGE_COUNT-1)
			{
			    count_data = 0;
				page++;
			}
        }
    }
}

/* 从指定 Flash 地址读取 1 字节数据，供 CRC 等校验计算使用 */
uint32_t FLASH_ReadByte(uint32_t address)
{
	return *(__IO uint8_t*)address; 
}
/* 对 Flash 中从 start_addr_bin 起、长度 len 的固件数据执行CRC32 计算/更新，用于验证 OTA 写入镜像的完整性 */
uint32_t crc32_update(uint32_t crc, const uint32_t start_addr_bin, uint32_t len) {
    crc = ~crc;  // 初始值取反（符合标准CRC32约定）
    for(int i =0 ;i<len;i++)
	{
        crc ^= FLASH_ReadByte(start_addr_bin+i);
        for(int i=0; i<8; i++) {
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
        }
    }
    return ~crc;  // 最终结果再次取反
}

/* 用于存放APP 升级标志位的 Flash 固定地址，OTA 完成后通过 app_flag_write() 写入该地址以通知启动流程“有新固件需要切换/升级” */
#define app_update_flag_addr 		0x0000E08
/* 用于存放IR-OTA 相关标志/魔数的 Flash 固定地址，OTA 完成后通过 app_flag_write1() 写入该地址以标记本次升级来源/状态（如写入 0x87654321） */
#define ir_ota_addr             0x0000E00

/* 实现 OTA 工作模式的主流程状态机：接收包→延时后发 ACK→超时退出→接收完成后做 CRC 校验→写升级标志→系统复位切换。 */
static void handle_work_mode_ir_ota(void)
{
    /* 接收完整数据包，设置发送标志位 */
    if (receive_ok_flag == 1)
    {
        receive_ok_flag = 0;
        timer_cnt = timer_ms();
        sent_ok = 0;
        if (message_id == 0x17)
        {
            OTA_Process(ir_rx_packet, sizeof(ir_rx_packet));
        }
    }

    /* 延时90ms，发送ACK反馈 */
    if (timer_elapsed(timer_cnt) > 90 && sent_ok == 0)
    {
        sent_ok = 1;
        if (message_id == 0x16)
        {
            FLASH_UnlockAllPages();
            /* 擦除25页FLASH，发送ACK反馈 */
            for (uint8_t i = 0; i < 25; i++)
            {
                FLASH_ErasePage(0x08004400 + i * 512);
            }
            FLASH_LockAllPages();
            Station_send_ack(0, message_id);
            recv_ota_data_ok = 0;
            set_led_twinkle_time(0xffffffff);
        }
        else if (message_id == 0x17)
        {
            Station_send_ack(6, message_id);
        }
        message_id = 0;
    }

    /* 超时处理 */
    if (timer_elapsed(timer_cnt) > 50 * 1000)
    {
        ota_ok = 0;
        base_work_mode = BASE_WORK_MODE_NORMAL;
        set_led_twinkle_time(0);
        bin_crc_data = 0;
    }

    /* OTA完成后的状态机及相关处理 */
    if (recv_ota_data_ok == 1)
    {
        switch (ota_ok)
        {
            case 1:
            {
                if (base_work_mode == BASE_WORK_MODE_NORMAL && robot_at_dock_state == ROBOT_STATE_NOT_AT_DOCK)
                {
                    ota_ok = 2;
                }
            }break;

            case 2:
            {
                if (robot_at_dock_state == ROBOT_STATE_AT_DOCK)           //bin文件校验
                {
                    if (crc32_update(0, start_add, Firmware_size) == bin_crc_data)   //bin文件校验正确
                    {
                        ota_ok = 3;
                    }
                    else                                //bin文件校验错误
                    {
                        ota_ok = 0;
                        base_work_mode = BASE_WORK_MODE_NORMAL;
                        set_led_twinkle_time(0);
                        bin_crc_data = 0;
                    }
                }
            }break;

            case 3:
            {
                app_flag_write(0xffffffff, app_update_flag_addr);
                app_flag_write1(0x87654321, ir_ota_addr);
                NVIC_SystemReset();
            }break;

            default:
                break;
        }
    }

    if (timer_elapsed(timeout) > 3600 * 1000 && timeout != 0)
    {
        set_led_twinkle_time(0);
        base_work_mode = BASE_WORK_MODE_NORMAL;
    }
}
