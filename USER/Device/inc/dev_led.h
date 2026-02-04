#ifndef __DEV_LED_H
#define __DEV_LED_H

#include <stdint.h>
void LED_ON(void);
void LED_OFF(void);
void led_set_pwm(uint8_t duty);
void led_breath(void);
void led_twinkle(void);

extern uint32_t led_twinkle_time;
void set_led_twinkle_time (uint32_t time);

void led_normal_mode_control(void);
void led_twinkle_time_process(void);

#endif /* __DEV_LED_H */
