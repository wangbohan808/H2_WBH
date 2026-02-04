#include "adc_cfg.h"
#include "cw32l010.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_adc.h"
#include "cw32l010_gpio.h"
#include "func_dust_collect.h"


void adc_capture_cfg(void)
{
	/* 打开GPIO与ADC的时钟，允许相关的寄存器设置 */
	__SYSCTRL_GPIOA_CLK_ENABLE();
	__SYSCTRL_ADC_CLK_ENABLE();
	ADC_WatchdogTypeDef ADC_WdtStructure = {0};

	/* 配置使用的ADC通道的引脚，设置为模拟输入 */
	PA04_ANALOG_ENABLE();       //ADC_IN4：（接收扫地机的信号）极片通讯、扫地机在位
	PB03_ANALOG_ENABLE();       //ADC_IN10

	/* 时钟分频、连续转换模式、开启两个转换通道 */
	ADC_WdtStructure.ADC_InitStruct.ADC_ClkDiv = ADC_Clk_Div8;
	ADC_WdtStructure.ADC_InitStruct.ADC_ConvertMode = ADC_ConvertMode_Continuous;
	ADC_WdtStructure.ADC_InitStruct.ADC_SQREns = ADC_SqrEns0to1;
	
	/* 将ADC的通道映射到特定的结果转化寄存器 */
	ADC_WdtStructure.ADC_InitStruct.ADC_IN0.ADC_InputChannel = ADC_InputCH10;
	ADC_WdtStructure.ADC_InitStruct.ADC_IN0.ADC_SampTime = ADC_SampTime390Clk;
	ADC_WdtStructure.ADC_InitStruct.ADC_IN1.ADC_InputChannel = ADC_InputCH4;
	ADC_WdtStructure.ADC_InitStruct.ADC_IN1.ADC_SampTime = ADC_SampTime390Clk;
	
	/* 设置模拟看门狗监控的ADC通道，设置上下阈值 */
	ADC_WdtStructure.ADC_WatchdogCHx = ADC_WATCHDOG_IN10;  
	ADC_WdtStructure.ADC_WatchdogOverHighIrq = ENABLE;
	ADC_WdtStructure.ADC_WatchdogUnderLowIrq = ENABLE;
	ADC_WdtStructure.ADC_WatchdogVth = 0x0F00;
	ADC_WdtStructure.ADC_WatchdogVtl = 0x0000;

	/* 看门狗功能需要基础的ADC配置，所以只需要声明一个看门狗的结构体，内部已包含ADC的结构体 */
	ADC_WatchdogInit(&ADC_WdtStructure);

	/* 使能ADC模拟看门狗中断，清除挂起位，防止初始化时误触发中断 */
	ADC_ITConfig(ADC_IT_AWDH|ADC_IT_AWDL, ENABLE);
	ADC_ClearITPendingAll();

	/* 在NVIC中使能ADC中断，使ADC的中断可以传递给CPU */
	NVIC_EnableIRQ(ADC_IRQn);
	
	/* 配置完成后使能ADC模块 */
	ADC_Enable();
	/* 软件启动ADC的转换 */
	ADC_SoftwareStartConvCmd(ENABLE);
}

/* 检测通道10是否触发看门狗，如果触发则进入看门狗中断处理函数 */
void ADC_IRQHandler(void)
{
	ADC_ClearITPendingAll();
    ac_frequency_calculate();
}
