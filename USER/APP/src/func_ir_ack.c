/* 使用硬件的基本功能（驱动）实现的逻辑功能，需要写在APP层 */

#include "func_ir_ack.h"
#include <stdint.h>
#include "tim_cfg.h"
#include "dev_ir.h"

static uint32_t ir_ack_code_modulate(uint8_t ir_code)
{
    uint32_t result = 0;
    uint8_t i = 0;
    for(i = 0;i < 8;i++)
    {
        /* 字节从高位到低位处理：1->100  0->10 */
        if(ir_code & 0x80)
        {
            result = ((result << 3) | 0x04);
        }
        else
        {
            result = ((result << 2) | 0x02);
        }
        ir_code <<= 1;  
    }
    return result;
}

/* 细节：数据头的八位发送结束，不会将灯关闭，数据尾发送结束，会将灯关闭：应该是担心数据头关灯会乱码 */
uint8_t sent_head(void)  
{
    uint8_t head_data=0xf8;      //11111000
    static uint8_t i=0;
    if((head_data<< i)&0x80)
    {
        IR_MIDDLE_LEFT_ON;
        IR_MIDDLE_RIGHT_ON;
    }
    else
    {
        IR_MIDDLE_LEFT_OFF;
        IR_MIDDLE_RIGHT_OFF;		  
    }
    i++;
    if(i>7)
    {
        i=0;
        return 1;
    }
    return 0;
}

uint8_t sent_tail(void)        //11110000
{
    uint8_t tail_data=0xf0;
    static uint8_t i=0;
    if((tail_data<<i)&0x80)
    {
        IR_MIDDLE_LEFT_ON;
        IR_MIDDLE_RIGHT_ON;
    }
    else
    {
        IR_MIDDLE_LEFT_OFF;
        IR_MIDDLE_RIGHT_OFF;
    }
    i++;
    if(i>7)
    {
      i=0;
      IR_MIDDLE_LEFT_OFF;
      IR_MIDDLE_RIGHT_OFF;
      return 1;
    }
    return 0;
}


/* 定义ack码的发送重复次数：动态设置，动态改变 */
uint8_t ir_send_count = 0;
void set_ir_send_count(uint8_t count)
{
    ir_send_count = count ;
}

/* 定义数组长度，影响处理位数 */
uint8_t ir_sent_bite = 0;
void set_ir_sent_bite(uint8_t count)
{
    ir_sent_bite = count;
}

/* 定义一个数组，向特定位写入数据，进而进行发送 */
uint8_t sent_buff[13] = {0};

/* 传递数组的头指针、长度：根据两信息，逐字节发送数据 */
uint8_t ir_sent_buf(uint8_t buf[], uint8_t len)
{
    static uint8_t index = 0;
    static uint32_t data = 0;
    static uint8_t i = 0;      // 当前处理数组对应的字节索引
    static uint8_t n = 0;      // 当前字节剩余待发送的位数
    uint8_t j = 0;
    
    if(ir_send_count == 0)
    {
        return 0;
    }
    
    switch(index)
    {
        case 0:  // 发送数据头
        {
            if(sent_head())
            {
                index = 1;
            }
        }
        break;
        
        case 1:  // 发送数据
        {
            /* 判断是否进行更新发送的字节：当前字节（数组的元素）的位数已发送完，准备下一个字节的编码 */
            if(n == 0)
            {
                /* 计数小于数组大小，还有字节需要发送 */
                if(i < len)  
                {
                    /* 计算当前字节编码后的位数 */
                    for(j = 0; j < 8; j++)
                    {
                        if((buf[i] << j) & 0x80)
                        {
                            n += 3;  // 1编码为3位
                        }
                        else
                        {
                            n += 2;  // 0编码为2位
                        }
                    }
                    
                    /* 编码当前字节 */
                    data = ir_ack_code_modulate(buf[i]);
                    
                    i++;  // 指向下一个字节
                }
                else  
                {
                    /* 所有字节发送完成，进入发送尾阶段 */
                    index = 2;  
                }
                
            }
            
            /* 字节编码后的数据发送流程 */
            if(n > 0)
            {
                n--;
                if(data & (0x01 << n))
                {
                    IR_MIDDLE_LEFT_ON;
                    IR_MIDDLE_RIGHT_ON;
                }
                else
                {
                    IR_MIDDLE_LEFT_OFF;
                    IR_MIDDLE_RIGHT_OFF;
                }
            }
        }
        break;
        
        case 2:  // 发送数据尾
        {
            if(sent_tail())
            {
                ir_send_count--;
                index = 0;
                i = 0;
                return 1;
            }
        }
        break;
    }
    
    /* 没有走到case0的数据尾：一帧（定义的一次完整数组）的数据没有发送完毕 */
    return 0;
}

void ir_send_ack(void)
{
    static uint8_t ir_send_ack_state = 0;
    static uint64_t time_cnt_reserve = 0;
    
    switch(ir_send_ack_state)
    {
        case 0:
        {
            time_cnt_reserve = timer_ms();
            if(ir_sent_buf(sent_buff,ir_sent_bite))
            {
                ir_send_ack_state = 1;
            }        
        }break;
        
        case 1:
        {
            ir_close_all();
            if(timer_elapsed(time_cnt_reserve) > 30)
            {
                 ir_send_ack_state = 0;
            }
        }            
    }
    
}


char version[2] = {1,25};

uint8_t soft_version = 3;




