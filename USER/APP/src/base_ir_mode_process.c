#include "base_ir_mode_process.h"
#include "func_ir_decoder.h"
#include "base_state.h"
#include "func_ir_ack.h"
#include "base_event.h"
#include "tim_cfg.h"
#include "func_dust_collect.h"
#include "dev_led.h"

static void handle_ir_normal_message(void);
static void handle_ir_version_request(void);
static void handle_ir_crc_info(void);
static void handle_work_mode_normal(void);
static void handle_work_mode_test(void);
static void handle_work_mode_ir_ota(void);

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
    if (ota_ok == 1)
    {
        set_ir_send_count(6);
    }
    else
    {
        set_ir_send_count(2);
    }
}

void base_ir_mode_process(void)
{
    ir_rx_packet_parse(ir_rx_packet,sizeof(ir_rx_packet));

    /* 统一处理非 OTA 模式下的红外普通消息 */
    if (base_work_mode != BASE_WORK_MODE_IR_OTA || message_id == 0x24 || message_id == 0x18)
    {
        if (receive_ok_flag == 1)
        {
            receive_ok_flag = 0;
            timer_cnt = timer_ms();
            send_ok = 0;
        }
        if (timer_elapsed(timer_cnt) >= 60 && send_ok == 0)
        {
            send_ok = 1;

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

/*===================== 小功能拆分实现 =====================*/

/* 普通红外命令：解析并创建对应事件 */
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

/* 版本请求消息处理（0x18） */
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

/* CRC 信息消息处理（0x24） */
static void handle_ir_crc_info(void)
{
    Station_send_ack(0, message_id);
    bin_crc_data = (uint32_t)ir_rx_packet[14] << 24 |
                   (uint32_t)ir_rx_packet[15] << 16 |
                   (uint32_t)ir_rx_packet[16] << 8  |
                   (uint32_t)ir_rx_packet[17];
}

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
                if (get_ir_enable_flag())
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
                if (get_ir_enable_flag())
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
                if (get_ir_enable_flag() == 0)
                {
                    FULL_GO_STEP = WORK_MODE_FULL_GO_IDEL;
                    set_base_work_mode(WORK_MODE_FULL_GO);
                    TEST_FULL_GO_SEC = TEST_IDEL;
                    set_light_twinkle_time(0);
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

static void base_mode_full_go_process(void);
/* 产测工作模式处理入口 */
static void handle_work_mode_test(void)
{
    set_ir_sent_bite(3);
    base_mode_full_go_process();
}

/* OTA 模式处理 */
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
            OTA_Process(recv_data_buf, sizeof(recv_data_buf));
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
            light_show_mode = 0;
            set_light_twinkle_time(0xffffffff);
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
        set_light_twinkle_time(0);
        bin_crc_data = 0;
    }

    /* OTA完成后的状态机及相关处理 */
    if (recv_ota_data_ok == 1)
    {
        switch (ota_ok)
        {
            case 1:
            {
                if (get_ir_enable_flag() == 1)
                {
                    ota_ok = 2;
                }
            }break;

            case 2:
            {
                if (get_ir_enable_flag() == 0)           //bin文件校验
                {
                    if (crc32_update(0, start_add, Firmware_size) == bin_crc_data)   //bin文件校验正确
                    {
                        ota_ok = 3;
                    }
                    else                                //bin文件校验错误
                    {
                        ota_ok = 0;
                        base_work_mode = BASE_WORK_MODE_NORMAL;
                        set_light_twinkle_time(0);
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
        set_light_twinkle_time(0);
        set_base_work_mode(WORK_MODE_IDEL);
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
        set_light_twinkle_time(0);
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
        set_light_twinkle_time(1 * 1000);
        light_show_mode = 0;
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
        set_base_work_mode(WORK_MODE_IDEL);
        TEST_FULL_GO_SEC = TEST_IDEL;
        base_event = 0;
        light_show_mode = 1;
    }
}

/* 电流测试事件 */
static void handle_test_current_event(void)
{
    if (get_base_event(BASE_EVENT_TEST_CURRENT) == 1)
    {
        sent_buff[0] = (uint8_t)((0xd << 4) + (get_changing_cur() >> 8));
        sent_buff[1] = (uint8_t)get_changing_cur();
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
            if (get_ir_enable_flag() == 0)
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
            if (light_twinkle_time == 0)
            {
                set_light_twinkle_time(1000);
                light_show_mode = !light_show_mode;
            }
        }break;

        default:
            break;
    }
}