#ifndef __FUNC_IR_DECODER_H
#define __FUNC_IR_DECODER_H

#include <stdint.h>

typedef enum
{
    IR_RESYNC,          // 等待信号进入同步
    IR_HEADER,          // 检测前导码
    IR_DATA_BIT_HIGH,   // 数据位高电平
    IR_DATA_BIT_LOW,    // 数据位低电平
} IR_DECODE_ST_E;

typedef struct
{
    IR_DECODE_ST_E state;    // 解码状态（同步/前导码/高电平/低电平）
    unsigned char timer;         // 一位解码的有效时间，用于超时判断（单位：100us）
    unsigned char count;        // 有效电平统计时间（主要统计低电平时间，单位：100us）
    unsigned char bits_count;   // 已解码位数统计（0-8）
    unsigned char decode_value; // 解码出的值（当前正在组装的字节）
} ir_decode_t;

#define DOCK_RESYNC_TICKS       (20 * 2)  
#define DOCK_HEADER_LOW         (25)           
#define DOCK_DATA_BIT_VALUE_1_TICK  (12)

void ir_detect_capture(void);
void ir_decoder_process(void);

extern uint8_t	ota_ok;

#endif /* __FUNC_IR_DECODER_H */
