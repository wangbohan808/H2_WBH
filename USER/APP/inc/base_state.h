#ifndef __BASE_STATE_H
#define __BASE_STATE_H

/* 极片通讯决定工作模式 */
typedef enum
{
    BASE_WORK_MODE_NORMAL = 0,
    BASE_WORK_MODE_TEST = 1,
    BASE_WORK_MODE_IR_OTA = 2,
}BASE_WORK_MODE_E;

/* 自检函数决定机器是否在位 */
typedef enum
{
    ROBOT_STATE_NOT_AT_DOCK = 0,   
    ROBOT_STATE_AT_DOCK     = 1, 
}ROBOT_AT_DOCK_STATE_E;

typedef enum
{
    DUST_BAG_STATE_UNSTALL = 0,    
    DUST_BAG_STATE_STALL   = 1,   
}DUST_BAG_STATE_E;


extern BASE_WORK_MODE_E base_work_mode;
extern ROBOT_AT_DOCK_STATE_E robot_at_dock_state;
extern DUST_BAG_STATE_E dust_bag_state;


#endif /* __BASE_STATE_H */
