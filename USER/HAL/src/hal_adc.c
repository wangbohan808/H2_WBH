#include "hal_adc.h"
#include <stdint.h>
#include "cw32l010.h"

uint16_t get_charging_cur(void)
{	
	return CW_ADC->RESULT1 ;
}
