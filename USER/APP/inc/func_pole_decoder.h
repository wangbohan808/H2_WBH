#ifndef __FUNC_POLE_DECODER_H
#define __FUNC_POLE_DECODER_H

#include <stdint.h>

/* 队列长度定义 */
#define POLE_QUEUE_LEN                64

/* ADC阈值定义 */
#define POLE_ADC_THRESHOLD            10

/* 连续电平计数阈值 */
#define CONSECUTIVE_LEVEL_COUNT       10

/* 低电平时间阈值定义（单位：根据实际计时单位调整，例如：ms或采样次数） */
#define POLE_LOW_TIME_200MS_MIN       1600  // 200ms模式最小时间
#define POLE_LOW_TIME_200MS_MAX       2400  // 200ms模式最大时间
#define POLE_LOW_TIME_400MS_MIN       3800  // 400ms模式最小时间
#define POLE_LOW_TIME_400MS_MAX       4200  // 400ms模式最大时间
#define POLE_LOW_TIME_600MS_MIN       5800  // 600ms模式最小时间
#define POLE_LOW_TIME_600MS_MAX       6200  // 600ms模式最大时间
#define POLE_LOW_TIME_800MS_MIN       7800  // 800ms模式最小时间
#define POLE_LOW_TIME_800MS_MAX       8200  // 800ms模式最大时间
#define POLE_LOW_TIME_1000MS_MIN      9800  // 1000ms模式最小时间
#define POLE_LOW_TIME_1000MS_MAX      10200 // 1000ms模式最大时间
#define POLE_LOW_TIME_1200MS_MIN      11800 // 1200ms模式最小时间
#define POLE_LOW_TIME_1200MS_MAX      12200 // 1200ms模式最大时间
#define POLE_LOW_TIME_2000MS_MIN      19800 // 2000ms模式最小时间
#define POLE_LOW_TIME_2000MS_MAX      20200 // 2000ms模式最大时间

/* 极片解码状态枚举 */
typedef enum
{
    POLE_RESYNC = 0,              // 重新同步状态
    POLE_HIGH,                    // 首先跳转到高电平状态，确保捕获的低电平时间是完整的
    POLE_LOW,                     // 低电平状态
    POLE_DUST_COLLECT,            // 集尘模式
    POLE_DUST_COLLECT_HIGH,       // 集尘模式高电平状态
    POLE_DUST_COLLECT_LOW,        // 集尘模式低电平状态
    POLE_DUST_COLLECT_COMPLETE,   // 集尘模式完成状态
    POLE_TEST_MODE,               // 产测模式
    POLE_TEST_MODE_RESYNC,        // 进入产测模式后，需要接收新的低电平时间，因而首先接收高电平进入同步态
    POLE_TEST_MODE_SELECT,        // 产测模式选择状态
} pole_decode_state_e;

/* 极片解码结构体 */
typedef struct
{
    pole_decode_state_e state;        // 当前解码状态
    uint16_t timer;                   // 计时器：记录某一状态的持续时间
} pole_decode_t;

/* 函数声明 */
void pole_detect_capture(void);
uint8_t pole_decoder_process(void);
void pole_decoder_reset(void);  // 重置极片解码器：重置状态机到POLE_RESYNC，并将所有状态时间统计归零

/* 各状态持续时间统计变量（单位：0.1ms） */
extern uint32_t pole_resync_time;                    // POLE_RESYNC 状态持续时间
extern uint32_t pole_high_time;                      // POLE_HIGH 状态持续时间
extern uint32_t pole_low_time;                       // POLE_LOW 状态持续时间
extern uint32_t pole_dust_collect_high_time;         // POLE_DUST_COLLECT_HIGH 状态持续时间
extern uint32_t pole_dust_collect_low_time;          // POLE_DUST_COLLECT_LOW 状态持续时间
extern uint32_t pole_dust_collect_complete_time;     // POLE_DUST_COLLECT_COMPLETE 状态持续时间
extern uint32_t pole_test_mode_time;                 // POLE_TEST_MODE 状态持续时间
extern uint32_t pole_test_mode_resync_time;          // POLE_TEST_MODE_RESYNC 状态持续时间
extern uint32_t pole_test_mode_select_time;          // POLE_TEST_MODE_SELECT 状态持续时间

#endif /* __FUNC_POLE_DECODER_H */
