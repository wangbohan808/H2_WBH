#ifndef __TIM_CFG_H
#define __TIM_CFG_H

#include <stdint.h>

void tim_ir_pwm_cfg(uint16_t arr,uint16_t psc);
void tim_100us_irq_cfg(uint16_t arr,uint16_t psc);

uint64_t timer_ms(void);
uint64_t timer_elapsed(uint64_t num);

#endif /* __TIM_CFG_H */
