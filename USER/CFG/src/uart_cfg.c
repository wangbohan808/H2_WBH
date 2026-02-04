#include "uart_cfg.h"
#include "cw32l010.h"
#include "cw32l010_uart.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"


/* 烧录引脚复用为uart串口 */
void debug_uart_init(void)
{
	/* 使用这一个类型的结构体，必须首先引用头文件，不然的话会报错 */
	UART_InitTypeDef UART_InitStructure;
	
	/* 将SWD引脚切换为GPIO模式 */
	GPIO_SWD2GPIO();
	
	/* 使能GPIO和UART时钟 */
	__SYSCTRL_GPIOA_CLK_ENABLE();
	__SYSCTRL_UART1_CLK_ENABLE();
	
	/* 配置PA08为UART1_TX复用功能 */
	PA08_DIGTAL_ENABLE();
	PA08_DIR_OUTPUT();
	PA08_PUSHPULL_ENABLE();
	PA08_AFx_UART1TXD();

	/* 配置PA07为UART1_RX（复用功能） */
	PA07_DIGTAL_ENABLE();
	PA07_DIR_INPUT();
	PA07_PUSHPULL_ENABLE();
	PA07_AFx_UART1RXD();
	
	/* 配置UART参数 */
	UART_StructInit(&UART_InitStructure);
	UART_InitStructure.UART_BaudRate = 115200;
	UART_InitStructure.UART_UclkFreq = 48000000;
	UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
	UART_Init(CW_UART1,&UART_InitStructure);
	
	/* 使能UART接收中断 */
	UART_ITConfig(CW_UART1,UART_IT_RC,ENABLE);
	
	/* 使能UART中断 */
	NVIC_EnableIRQ(UART1_IRQn);
}
