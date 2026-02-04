#ifndef __BASE_DUST_MODE_H
#define __BASE_DUST_MODE_H

#include <stdint.h>

typedef enum
{
    DUST_MODE_POLE_COMM = 0,
    DUST_MODE_ROBOT_UNDOCK = 1,
} dust_collect_mode_e;

//#define POLE_COMM_VALID_PERIOD_MS     (2UL * 24UL * 60UL * 60UL * 1000UL)
#define POLE_COMM_VALID_PERIOD_MS     (60UL * 1000UL)
#define UNDOCK_MIN_INTERVAL_MS        (3UL * 60UL * 1000UL)
#define UNDOCK_MAX_INTERVAL_MS        (3UL * 60UL * 60UL * 1000UL)

void dust_mode_init(void);
void dust_mode_on_pole_comm(void);
void dust_mode_check_and_switch(void);
uint8_t dust_mode_record_undock(void);

extern dust_collect_mode_e current_dust_mode;
extern uint64_t last_undock_time;

#endif
