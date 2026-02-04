#include "base_dust_mode.h"
#include "tim_cfg.h"
#include <stdint.h>

dust_collect_mode_e current_dust_mode = DUST_MODE_POLE_COMM;

static uint64_t last_pole_comm_time = 0;

uint64_t last_undock_time = 0;

void dust_mode_init(void)
{
    current_dust_mode = DUST_MODE_POLE_COMM;
    last_pole_comm_time = timer_ms();
    last_undock_time = timer_ms();
}

void dust_mode_on_pole_comm(void)
{
    last_pole_comm_time = timer_ms();
    current_dust_mode = DUST_MODE_POLE_COMM;
}

void dust_mode_check_and_switch(void)
{
    if(current_dust_mode == DUST_MODE_POLE_COMM)
    {
        uint32_t elapsed = timer_elapsed(last_pole_comm_time);
        
        if(elapsed > POLE_COMM_VALID_PERIOD_MS)
        {
            current_dust_mode = DUST_MODE_ROBOT_UNDOCK;
        }
    }
}

uint8_t dust_mode_record_undock(void)
{
    if(current_dust_mode != DUST_MODE_ROBOT_UNDOCK)
    {
        return 0;
    }
    
    uint32_t interval = timer_elapsed(last_undock_time);
    
    if(interval >= UNDOCK_MIN_INTERVAL_MS && 
       interval <= UNDOCK_MAX_INTERVAL_MS)
    {
        return 1;
    }
    
    return 0;
}
