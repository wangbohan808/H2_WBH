#include "hal_tim.h"
#include <stddef.h>

/* ==================== 定时器回调结构体定义 ==================== */

/* 每个定时器实例可注册的回调函数最大数量 */
#define HAL_TIMER_CB_MAX 10

/* 定时器具体某一个回调函数结构体：最主体的部分就是间隔特定时间执行一次回调 */
typedef struct hal_timer_callback_item_t
{
    int execute_count;                           /* 执行计数器：记录当前已执行的次数 */
    int trigger_interval;                        /* 触发间隔：每多少次调用才执行一次回调 */
    hal_timer_callback_t callback_func;          /* 回调函数指针 */
} hal_timer_callback_item_t;

/* 每个定时器实例可注册多个回调函数 */
typedef struct hal_timer_manager_t
{
    hal_timer_callback_item_t callbacks[HAL_TIMER_CB_MAX];  /* 回调函数数组 */
    uint8_t registered_count;                               /* 已注册的回调函数数量 */
} hal_timer_manager_t;

/* 定时器的最大数量 */
#define HAL_TIMER_COUNT_MAX 4
/* 定时器管理器实例数组，管理多个定时器 */
static hal_timer_manager_t hal_timer_manager[HAL_TIMER_COUNT_MAX] = {0};

/* 三个参数的含义：注册回调函数到哪一个定时器，使用回调函数传入具体的执行任务，间隔多少次执行一次回调*/
bool hal_timer_task_register(uint8_t timer_index, hal_timer_callback_t callback, int trigger_interval)
{  
    /* 参数检查 */
    if (timer_index >= HAL_TIMER_COUNT_MAX || callback == NULL || trigger_interval <= 0)
    {
        return false;
    }
    
    /* 局部变量接收定时器管理数组 */
    hal_timer_manager_t *timer_handle = &hal_timer_manager[timer_index];
    
    /* 使用'for循环'以及'if判断'，查找空闲的回调槽位 */
    for (int i = 0; i < HAL_TIMER_CB_MAX; i++)
    {
        if (timer_handle->callbacks[i].callback_func == NULL)
        {
            /* 找到空闲槽位，写入回调的三个成员变量，相当于完成注册 */
            timer_handle->callbacks[i].callback_func = callback;
            timer_handle->callbacks[i].execute_count = 0;
            timer_handle->callbacks[i].trigger_interval = trigger_interval;
            /* 管理指定定时器实例的回调函数数量 */
            timer_handle->registered_count++;
            return true;
        }
    }   
    /* 回调数组已满 */
    return false;
}

/* 指定调用的定时器，在对应的中断中按定义调用此定时器的回调函数 */
void hal_timer_run(uint8_t timer_index)
{
    if (timer_index >= HAL_TIMER_COUNT_MAX)
    {
        return;
    }

    hal_timer_manager_t *timer_handle = &hal_timer_manager[timer_index];
    
    for (int i = 0; i < timer_handle->registered_count; i++)
    {
        timer_handle->callbacks[i].execute_count++;
        
        if (timer_handle->callbacks[i].execute_count >= 
            timer_handle->callbacks[i].trigger_interval)
        {
            if (timer_handle->callbacks[i].callback_func != NULL)
            {
                // 直接调用，不传参数
                timer_handle->callbacks[i].callback_func();
            }
            
            timer_handle->callbacks[i].execute_count = 0;
        }
    }
}

