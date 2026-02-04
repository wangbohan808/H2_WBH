#ifndef __HAL_TIM_H
#define __HAL_TIM_H

#include <stdint.h>
#include <stdbool.h>

/* 定时器回调函数指针类型为hal_timer_callback_t，后续直接向结构体传入成员函数 */
typedef void (*hal_timer_callback_t)(void);

/* 函数声明 */
bool hal_timer_task_register(uint8_t timer_index, hal_timer_callback_t callback, int trigger_interval);
void hal_timer_run(uint8_t timer_index);

#endif /* __HAL_TIM_H */
