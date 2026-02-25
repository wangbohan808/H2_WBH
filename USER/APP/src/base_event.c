

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

void clear_all_event()
{
    base_event = 0;
}

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

/* 定义提供初始化列表的数组，可以不写大小 */
/* 红外命令表的索引，与BASE_EVENT_E对应，这样后续检测命令在命令表中的元素位置，相当于事件的编号 */
const IR_CDM_E ir_cmd_table[] = {
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
