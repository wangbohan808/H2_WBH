#ifndef __UTILS_QUEUE_H
#define __UTILS_QUEUE_H

#include <stdint.h>


///循环队列结构体定义
typedef struct queue_circular_t
{
    /* 在32位MCU中，所有的指针都是32位，不需要声明，uint8_t* 声明的是指针元素的类型 */
    uint8_t * data_buffer;   //数据缓存
    uint16_t write_ptr;      //当前写地址（数组索引）
    uint16_t read_ptr;       //当前读地址（数组索引）
    uint16_t queue_len;      //队列缓存长度
} queue_circular_t;

/* 队列操作函数 */
void queue_circular_reset(queue_circular_t * queue);
void queue_circular_init(queue_circular_t * queue, uint8_t * queue_buffer, uint16_t queue_length);
uint16_t queue_circular_get_datalength(queue_circular_t * queue);
uint8_t queue_circular_is_full(queue_circular_t * queue);
uint8_t queue_circular_is_empty(queue_circular_t * queue);
uint16_t queue_circular_get_space(queue_circular_t * queue);
void queue_circular_put(queue_circular_t * queue, uint8_t value);
uint8_t queue_circular_get(queue_circular_t * queue);

#endif /* __UTILS_QUEUE_H */
