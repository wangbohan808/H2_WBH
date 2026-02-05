#ifndef __BASE_EVENT_H
#define __BASE_EVENT_H

#include <stdint.h>

/* 事件枚举定义 */
typedef enum
{
    /* 系统事件 */
    BASE_EVENT_SYS_INIT          = 0,        // 系统初始化完成
    
    /* 极片解码事件 */
    BASE_EVENT_POLE_SYNC         = 1,        // 极片同步完成
    BASE_EVENT_POLE_DUST_START   = 2,        // 极片集尘模式开始
    BASE_EVENT_POLE_DUST_COMPLETE= 3,        // 极片集尘模式完成
    BASE_EVENT_POLE_TEST_START   = 4,        // 产测模式开始
    BASE_EVENT_POLE_TEST_SELECT  = 5,        // 产测模式选择
    
    /* 机器人状态事件 */
    BASE_EVENT_ROBOT_UNDOCK      = 6,        // 机器人离座
    BASE_EVENT_ROBOT_DOCK        = 7,        // 机器人在座
    
    /* 集尘控制事件 */
    BASE_EVENT_DUST_START        = 8,        // 集尘开始（风机启动）
    BASE_EVENT_DUST_COMPLETE     = 9,        // 集尘完成（风机停止）
    
    /* 尘袋状态事件 */
    BASE_EVENT_DUST_BAG_INSTALL  = 10,       // 尘袋安装
    BASE_EVENT_DUST_BAG_REMOVE   = 11,       // 尘袋移除
    
    /* 工作模式事件 */
    BASE_EVENT_MODE_NORMAL       = 12,       // 进入正常模式
    BASE_EVENT_MODE_TEST         = 13,       // 进入产测模式
    BASE_EVENT_MODE_SWITCH       = 14,       // 集尘模式切换（极片通讯<->机器人离座）
    
    /* 红外通讯事件 */
    BASE_EVENT_IR_GUIDE_COMPLETE = 15,       // 红外引导码发送完成
    BASE_EVENT_IR_ACK_COMPLETE   = 16,       // 红外ACK发送完成
    
    /* 预留事件 */
    BASE_EVENT_RESERVE           = 31,       // 预留
} BASE_EVENT_E;

/* 函数声明 */
void create_base_event(BASE_EVENT_E event);
void clear_base_event(BASE_EVENT_E event);
uint8_t get_base_event(BASE_EVENT_E event);
void clear_all_event(void);

/* 外部变量声明 */
extern uint32_t base_event;

#endif /* __BASE_EVENT_H */
