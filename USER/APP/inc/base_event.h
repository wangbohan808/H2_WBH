#ifndef __BASE_EVENT_H
#define __BASE_EVENT_H

#include <stdint.h>

/* 配合后续红外指令集，接收创建事件 */
typedef enum
{
    BASE_EVENT_RESERVE          = 0,        //预留
    BASE_EVENT_VACUUM_START     = 1,        //集尘开始
    BASE_EVENT_TEST_START       = 2,        //进入测试模式
	BASE_EVENT_TEST_START1      = 3,
	
	BASE_EVENT_TEST_START2      = 4,
    BASE_EVENT_TEST_VACUUN_START= 5,        //开启风机测试
	BASE_EVENT_TEST_VACUUN_START15 = 6,
    BASE_EVENT_TEST_VACUUN_END  = 7,        //结束风机测试
	
    BASE_EVENT_TEST_DUSTBUG     = 8,        //开启尘袋测试
	BASE_EVENT_TEST_LED_ON      = 9,
	BASE_EVENT_TEST_CURRENT     =10,
	BASE_EVENT_TEST_IR_OK       =11,
	
	BASE_EVENT_TEST_IR_ERR      =12,
    BASE_EVENT_TEST_END         =13,        //推出测试模式
	BASE_EVENT_CHARGE_COMPLETE  =14,        //充电完成
	BASE_EVENT_VACUUM_RM_STAR   =15,
	BASE_EVENT_CHARGE_ON        =16,
}BASE_EVENT_E;

/* 红外指令集，.c文件中数组索引与上述事件的值对应；
捕捉到红外指令，找到数组索引，就可以创建索引值的事件 */
typedef enum
{
    RESERVE                  = 0X0,
    IR_CMD_VACUUM_START      = 0xb200,      
    IR_CMD_TEST_START        = 0XA55A,      
	IR_CMD_TEST_START1       = 0X8888,
	IR_CMD_TEST_START2       = 0X6666,
    IR_CMD_TEST_VACUUM_ON    = 0X0101,
	IR_CMD_TEST_VACUUM_ON_15 = 0X0102,
    IR_CMD_TEST_VACUUM_OFF   = 0Xffff,
    IR_CMD_TEST_DUSTBUG      = 0X0200,
	IR_CMD_TEST_LED_ON       = 0X0301,
	IR_CMD_TEST_CURRENT      = 0X1000,
	IR_CMD_TEST_IR_OK        = 0X0401,
	IR_CMD_TEST_IR_ERR       = 0X0402,
    IR_CMD_TEST_END          = 0XABBA,
	IR_CMD_CHARGE_COMPLETE   = 0X0164,
	IR_CMD_VACUUM_RM_STAR    = 0X0e00,
	IR_CMD_CHARGE_ON         = 0Xba00,
}IR_CDM_E;

/* 事件标志位操作接口 */
void create_base_event(BASE_EVENT_E event);
void clear_base_event(BASE_EVENT_E event);
uint8_t get_base_event(BASE_EVENT_E event);
void clear_all_event(void);

/* 红外命令检查接口：返回命令在表中的索引（事件编号），0 表示未找到或预留 */
uint8_t ir_check_is_cmd(uint16_t ir_code);

extern uint32_t base_event;

#endif /* __BASE_EVENT_H */



