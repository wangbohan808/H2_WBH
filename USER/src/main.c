#include "main.h"
#include "cw32l010.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_systick.h"

#include "func_pole_decoder.h"
#include "func_ir_decoder.h"
#include "func_docking_guide.h"
#include "func_dust_collect.h"
#include "func_ir_ack.h"
#include "base_self_det.h"
#include "base_dust_mode.h"

#include "tim_cfg.h"
#include "adc_cfg.h"
#include "led_cfg.h"
#include "iwdg_cfg.h"
#include "fan_cfg.h"
#include "ir_cfg.h"

#include "hal_gpio.h"
#include "uart_cfg.h"
#include "hal_uart.h"
#include "hal_tim.h"
#include "dev_led.h"




static void Hardware_init(void);
static void Timer_Task_Init(void);

#if ENABLE_DEBUG_PRINT

uint64_t power_on_time = 0;
uint8_t uart_init_flag = 0;  

#endif


int32_t main(void)
{   
    /* 使用HSI提供系统主时钟，并且设置为不分频 */
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1);
    /* 这样配置相当于每1ms触发一次系统滴答中断 */
    InitTick(48000000);
    /* 初始化硬件 */
    Hardware_init();   
    /* 初始化时间片任务 */
    Timer_Task_Init();
    /* 初始化集尘模式管理模块 */
    dust_mode_init();
    
#if ENABLE_DEBUG_PRINT
    
    /* 记录上电时间 */
    power_on_time = timer_ms();

#endif
    
    /* 进入主循环 */
    while (1)
    {
#if ENABLE_DEBUG_PRINT
        
        /* 上电20s后才将SWD烧录端口复用为打印端口 */
        if(uart_init_flag == 0 && timer_elapsed(power_on_time) >= 20000)
        {
            debug_uart_init();
            uart_init_flag = 1;
            printf("UART initialized after 20s\r\n");
        }
#endif
        
        /* 检查集尘模式并自动切换（如果模式一超过2天未收到通讯，切换到模式二） */
        dust_mode_check_and_switch();        
        pole_decoder_process();

        ir_decoder_process();
        
        /* 根据标志位，控制风机的转动状态 */
        if(need_duty == 1)
        {
            dust_absorption_ctrl();
        }
        
        
        /* 检测adc值，转变机器人离座状态 */
        base_self_detect();
    }
}


void SYSCTRL_Configuration(void)
{
    // 使能 FLASH 时钟
    __SYSCTRL_FLASH_CLK_ENABLE();

    // 使能 GPIOA / GPIOB 时钟
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();
}


static void Hardware_init(void)
{
    SYSCTRL_Configuration();
    /* 配置间隔100us触发一次中断回调 */
    tim_100us_irq_cfg(100-1,48-1);

    /* 初始化LED */
    led_init();

    /* 捕获充电电流的ADC */
    adc_capture_cfg();

    /* 发送引导码与通讯码 */
    ir_output_init();

    /* 通讯与产测的时候代替极片接收命令与反馈*/
    ir_input_init();

    /* 配置控制风机转动的端口 */
    fan_gpio_init();

    /* 配置尘袋在位检测端口 */
    gpio_input_cfg(CW_GPIOA , GPIO_PIN_0); 

    /* 配置看门狗：防止程序跑飞 */
    iwdg_init();    
}

/* 时间片任务初始化：统一注册所有在中断中执行的任务 */
static void Timer_Task_Init(void)
{
    /* ==================== 注册100us定时器的任务 ==================== */
    
    /* 红外引导码发送任务：每100us执行一次（trigger_interval = 1） */
    hal_timer_task_register(HAL_TIMER_INDEX_100US, ir_docking_guide_send, 1);
    
    /* 极柱检测捕获任务：每100us执行一次（trigger_interval = 1） */
    hal_timer_task_register(HAL_TIMER_INDEX_100US, pole_detect_capture, 1);

    /* 红外检测捕获任务：每100us执行一次（trigger_interval = 1） */
    hal_timer_task_register(HAL_TIMER_INDEX_100US, ir_detect_capture, 1);
    
    /* LED正常模式控制任务：每100us执行一次（trigger_interval = 1） */
    hal_timer_task_register(HAL_TIMER_INDEX_100US, led_normal_mode_control, 1);
    
    /* ==================== 注册1ms定时器的任务 ==================== */
    
    /* LED闪烁时间处理任务：每1ms执行一次（trigger_interval = 1） */
    hal_timer_task_register(HAL_TIMER_INDEX_1MS, led_twinkle_time_process, 1);
    
    /* 红外应答发送任务：每1ms执行一次（trigger_interval = 1） */
    hal_timer_task_register(HAL_TIMER_INDEX_1MS, ir_send_ack, 1);
}


#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
    // 用户可在此添加错误处理，例如打印出错文件和行号
    // 例如：printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
    while (1); // 死循环，暂停程序
}
#endif

