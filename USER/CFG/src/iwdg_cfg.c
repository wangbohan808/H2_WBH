#include "iwdg_cfg.h"
#include "cw32l010.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_iwdt.h"


void iwdg_init(void)
{
	__SYSCTRL_IWDT_CLK_ENABLE(); 
	IWDT_InitTypeDef IWDT_InitStruct ={0};

	IWDT_InitStruct.IWDT_ITState = ENABLE;
	IWDT_InitStruct.IWDT_OverFlowAction = IWDT_OVERFLOW_ACTION_RESET;
	IWDT_InitStruct.IWDT_Pause = IWDT_SLEEP_PAUSE ;
	IWDT_InitStruct.IWDT_Prescaler = IWDT_Prescaler_DIV512;     // 默认设置为最大分频，计时间隔为15.6ms
	IWDT_InitStruct.IWDT_WindowValue = 0xFFF;          // 关闭窗口看门狗的功能
	IWDT_InitStruct.IWDT_ReloadValue = IWDT_2_SECS;

	// IWDT的时钟为LSI，启动IWDT前LSI必须有效
	if (CW_SYSCTRL->CR1_f.LSIEN == 0)
	{
		SYSCTRL_LSI_Enable();
	}  
	IWDT_Init(&IWDT_InitStruct);
	IWDT_Cmd();
	IWDT_Refresh();   
}
