#include "utils_queue.h"
#include <stddef.h>  

/* ========================== 重置队列、创建队列后进行初始化队列 ========================== */

/**
 * @brief 重置队列，将读写指针都置为0
 * @param queue 队列结构体指针
 */
void queue_circular_reset(queue_circular_t * queue)
{
    if (queue == NULL)
    {
        return;
    }
    queue->read_ptr = 0;
    queue->write_ptr = 0;
}

/**
 * @brief 初始化队列
 * @param queue 队列结构体指针
 * @param queue_buffer 数据缓冲区指针
 * @param queue_length 队列长度（缓冲区大小）
 */
void queue_circular_init(queue_circular_t * queue, uint8_t * queue_buffer, uint16_t queue_length)
{
    if (queue == NULL || queue_buffer == NULL || queue_length == 0)
    {
        return;
    }
    queue->data_buffer = queue_buffer;
    queue->queue_len = queue_length;
    queue_circular_reset(queue);
}


/* ========================== 队列状态查询函数 ========================== */

/**
 * @brief 获取队列中数据的长度
 * @param queue 队列结构体指针
 * @return 队列中数据的长度（元素个数）
 */
uint16_t queue_circular_get_datalength(queue_circular_t * queue)
{
    if (queue == NULL)
    {
        return 0;
    }
    
    // 循环数组作为队列，数据长度通过读写指针计算得到
    if (queue->write_ptr == queue->read_ptr)
    {
        // 读写指针重合，队列为空
        return 0;
    }
    else if (queue->write_ptr > queue->read_ptr)
    {
        // 写指针在读指针前面（未发生回绕）
        return queue->write_ptr - queue->read_ptr;
    }
    else
    {
        // 写指针在读指针后面（发生了回绕）
        return queue->write_ptr + (queue->queue_len - queue->read_ptr);
    }
}

/**
 * @brief 判断队列是否已满
 * @param queue 队列结构体指针
 * @return 1-队列已满，0-队列未满
 */
uint8_t queue_circular_is_full(queue_circular_t * queue)
{
    if (queue == NULL)
    {
        return 0;
    }
    
    // 循环数组作为队列，通过读写指针计算判断
    if (queue->write_ptr == queue->read_ptr)
    {
        // 读写指针重合，队列为空（不是满）
        return 0;
    }
    else if (queue->write_ptr > queue->read_ptr)
    {
        // 写指针在读指针前面（未发生回绕）
        // 队列满的条件：写指针与读指针的差值等于队列长度-1
        return (queue->write_ptr - queue->read_ptr) >= (queue->queue_len - 1) ? 1 : 0;
    }
    else
    {
        // 写指针在读指针后面（发生了回绕）
        // 队列满的条件：读指针与写指针的差值等于1
        return (queue->read_ptr - queue->write_ptr) <= 1 ? 1 : 0;
    }
}

/**
 * @brief 判断队列是否为空
 * @param queue 队列结构体指针
 * @return 1-队列为空，0-队列不为空
 */
uint8_t queue_circular_is_empty(queue_circular_t * queue)
{
    if (queue == NULL)
    {
        return 1;
    }
    return (queue->write_ptr == queue->read_ptr) ? 1 : 0;
}

/**
 * @brief 获取队列可用空间
 * @param queue 队列结构体指针
 * @return 队列可用空间（可存储的元素个数）
 * @note 队列可用长度 = 数组长度 - 1，因为写指针与读指针重合记为空
 */
uint16_t queue_circular_get_space(queue_circular_t * queue)
{
    if (queue == NULL)
    {
        return 0;
    }
    return (queue->queue_len - 1 - queue_circular_get_datalength(queue));
}


/* ========================== 队列数据操作函数 ========================== */

/**
 * @brief 向队列写入一个数据
 * @param queue 队列结构体指针
 * @param value 要写入的数据
 * @note 如果队列已满，会覆盖最旧的数据（建议先检查队列是否满）
 */
void queue_circular_put(queue_circular_t * queue, uint8_t value)
{
    if (queue == NULL || queue->data_buffer == NULL)
    {
        return;
    }
    
    // 写入数据
    queue->data_buffer[queue->write_ptr] = value;
    
    // 更新写指针
    queue->write_ptr++;
    if (queue->write_ptr >= queue->queue_len)
    {
        queue->write_ptr = 0;  // 回绕到数组开头
    }
}

/**
 * @brief 从队列读取一个数据
 * @param queue 队列结构体指针
 * @return 读取到的数据，如果队列为空则返回0
 * @warning 使用前必须判断队列是否为空，否则可能读取到无效数据
 */
uint8_t queue_circular_get(queue_circular_t * queue)
{
    uint8_t data = 0;
    
    if (queue == NULL || queue->data_buffer == NULL)
    {
        return 0;
    }
    
    // 读取数据
    data = queue->data_buffer[queue->read_ptr];
    
    // 更新读指针
    queue->read_ptr++;
    if (queue->read_ptr >= queue->queue_len)
    {
        queue->read_ptr = 0;  // 回绕到数组开头
    }
    
    return data;
}
