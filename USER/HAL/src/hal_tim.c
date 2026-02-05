#include "hal_tim.h"
#include <stddef.h>

/* ==================== 定时器回调链式结构体定义 ==================== */

/* 每个定时器实例可注册的回调函数最大数量 */
#define HAL_TIMER_CB_MAX 10

/* 定时器回调链表节点结构体 */
typedef struct hal_timer_callback_node_t
{
    int execute_count;                           /* 执行计数器 */
    int trigger_interval;                        /* 触发间隔 */
    hal_timer_callback_t callback_func;          /* 回调函数指针 */
    struct hal_timer_callback_node_t *next;      /* 指向下一个节点 */
} hal_timer_callback_node_t;

/* 定时器管理器：使用链表管理回调函数 */
typedef struct hal_timer_manager_t
{
    hal_timer_callback_node_t *head;             /* 链表头指针 */
    uint8_t registered_count;                    /* 已注册的回调函数数量 */
} hal_timer_manager_t;

/* 定时器的最大数量 */
#define HAL_TIMER_COUNT_MAX 4

/* 静态内存池：预分配节点，避免动态内存分配 */
static hal_timer_callback_node_t node_pool[HAL_TIMER_COUNT_MAX * HAL_TIMER_CB_MAX];
static uint8_t node_pool_used[HAL_TIMER_COUNT_MAX * HAL_TIMER_CB_MAX] = {0};

/* 定时器管理器实例数组 */
static hal_timer_manager_t hal_timer_manager[HAL_TIMER_COUNT_MAX] = {0};

/* 从内存池分配一个节点 */
static hal_timer_callback_node_t* node_alloc(void)
{
    for (int i = 0; i < HAL_TIMER_COUNT_MAX * HAL_TIMER_CB_MAX; i++)
    {
        if (node_pool_used[i] == 0)
        {
            node_pool_used[i] = 1;
            node_pool[i].next = NULL;
            node_pool[i].callback_func = NULL;
            node_pool[i].execute_count = 0;
            node_pool[i].trigger_interval = 0;
            return &node_pool[i];
        }
    }
    return NULL;  /* 内存池已满 */
}

/* 释放节点到内存池 */
void node_free(hal_timer_callback_node_t *node)
{
    if (node == NULL)
    {
        return;
    }
    
    for (int i = 0; i < HAL_TIMER_COUNT_MAX * HAL_TIMER_CB_MAX; i++)
    {
        if (&node_pool[i] == node)
        {
            node_pool_used[i] = 0;
            return;
        }
    }
}

/* 注册回调函数到指定定时器 */
bool hal_timer_task_register(uint8_t timer_index, hal_timer_callback_t callback, int trigger_interval)
{  
    /* 参数检查 */
    if (timer_index >= HAL_TIMER_COUNT_MAX || callback == NULL || trigger_interval <= 0)
    {
        return false;
    }
    
    hal_timer_manager_t *timer_handle = &hal_timer_manager[timer_index];
    
    /* 检查是否已注册 */
    hal_timer_callback_node_t *current = timer_handle->head;
    while (current != NULL)
    {
        if (current->callback_func == callback)
        {
            return false;  /* 已存在，不重复注册 */
        }
        current = current->next;
    }
    
    /* 检查数量限制 */
    if (timer_handle->registered_count >= HAL_TIMER_CB_MAX)
    {
        return false;  /* 已达到最大数量 */
    }
    
    /* 分配新节点 */
    hal_timer_callback_node_t *new_node = node_alloc();
    if (new_node == NULL)
    {
        return false;  /* 内存池已满 */
    }
    
    /* 初始化节点 */
    new_node->callback_func = callback;
    new_node->execute_count = 0;
    new_node->trigger_interval = trigger_interval;
    
    /* 头插法：插入到链表头部 */
    new_node->next = timer_handle->head;
    timer_handle->head = new_node;
    timer_handle->registered_count++;
    
    return true;
}

/* 在中断中执行定时器回调函数 */
void hal_timer_run(uint8_t timer_index)
{
    if (timer_index >= HAL_TIMER_COUNT_MAX)
    {
        return;
    }

    hal_timer_manager_t *timer_handle = &hal_timer_manager[timer_index];
    hal_timer_callback_node_t *current = timer_handle->head;
    
    /* 遍历链表，执行所有回调函数 */
    while (current != NULL)
    {
        current->execute_count++;
        
        if (current->execute_count >= current->trigger_interval)
        {
            if (current->callback_func != NULL)
            {
                current->callback_func();
            }
            current->execute_count = 0;
        }
        
        current = current->next;
    }
}

