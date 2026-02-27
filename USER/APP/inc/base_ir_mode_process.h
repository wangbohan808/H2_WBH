#ifndef __BASE_IR_MODE_PROCESS_H
#define __BASE_IR_MODE_PROCESS_H

#include <stdint.h>


typedef enum          //进入老化模式步骤
{
    WORK_MODE_FULL_GO_IDEL     = 0,
    WORK_MODE_FULL_GO_FIRST    = 1,     //步骤1
    WORK_MODE_FULL_GO_SECOND   = 2,     //步骤2
    WORK_MODE_FULL_GO_END      = 3,     //步骤3
}WORK_MODE_FULL_GO_STEP;


typedef enum          //常常模式测试项目 
{
    TEST_IDEL            = 0,
	TEST_IR              = 1,
    TEST_VACUUN_TEST     = 2,
    TEST_DUSTBUG_TEST    = 3,     
    TEST_LED_ON_TEST     = 4,     
	TEST_CURRENT_        = 5,
    TEST_END             = 6,	
}TEST_FULL_GO;

void base_ir_mode_process(void);
void Station_send_ack(uint8_t ack, uint8_t cmd1);

#endif /* __BASE_IR_MODE_PROCESS_H */