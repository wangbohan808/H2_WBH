#ifndef __FUNC_DUST_COLLECT_H
#define __FUNC_DUST_COLLECT_H

#include <stdint.h>

void ac_frequency_calculate(void);
void dust_absorption_ctrl(void);

void delay_10us(uint16_t us);

extern uint8_t need_duty;

extern uint8_t ac_frequency;

extern uint64_t dust_collect_time_cnt;

extern uint32_t dust_bag_time;

extern uint8_t dust_absorption_time;

uint32_t get_time_for_dust_finish(void);


#endif /* __FUNC_DUST_COLLECT_H */
