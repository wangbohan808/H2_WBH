#include "base_event.h"
#include <stdint.h>

/* 32位最多指代32个事件 */
uint32_t base_event = BASE_EVENT_RESERVE;

void create_base_event(BASE_EVENT_E event)
{
    base_event |= (1 << event);
}

void clear_base_event(BASE_EVENT_E event)
{
    base_event &= ~(1 << event);
}

/* 获取事件（指定位是0/1），并清除事件 */
uint8_t get_base_event(BASE_EVENT_E event)
{
    unsigned char ret = 0;
    ret = ((base_event >> event) & 1);
    base_event &= ~(1 << event);	
    return ret;
}

void clear_all_event(void)
{
    base_event = 0;
}


/* 定义提供初始化列表的数组，可以不写大小 */
/* 红外命令表的索引，与BASE_EVENT_E对应，这样后续检测命令在命令表中的元素位置，相当于事件的编号 */
static const IR_CDM_E ir_cmd_table[] = {
    RESERVE,
    IR_CMD_VACUUM_START,
    IR_CMD_TEST_START,
	IR_CMD_TEST_START1,
	IR_CMD_TEST_START2,
    IR_CMD_TEST_VACUUM_ON,
	IR_CMD_TEST_VACUUM_ON_15,
    IR_CMD_TEST_VACUUM_OFF,
    IR_CMD_TEST_DUSTBUG,
	IR_CMD_TEST_LED_ON,
	IR_CMD_TEST_CURRENT,	
	IR_CMD_TEST_IR_OK,
	IR_CMD_TEST_IR_ERR,
    IR_CMD_TEST_END,
	IR_CMD_CHARGE_COMPLETE,
	IR_CMD_VACUUM_RM_STAR,
	IR_CMD_CHARGE_ON,
};

/* 检查红外命令是否在命令表中，并返回在红外命令表中的位置，相当于事件的编号 */
uint8_t ir_check_is_cmd(uint16_t ir_code)
{
    for(uint8_t i=0;i<sizeof(ir_cmd_table)/sizeof(ir_cmd_table[0]);i++)
    {
        if(ir_code == ir_cmd_table[i])
        {
            return i;
        }
    }
    return 0;
}
