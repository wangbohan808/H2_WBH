#include "func_docking_guide.h"
#include "stdint.h"
#include "dev_ir.h"

/* 1->10000 0->11110，先进来的数据左移5位，8位原始数据->40位解析后的数据 */
static uint64_t ir_guide_code_modulate(uint8_t ir_code)
{
	uint8_t i = 0;
	uint64_t ret = 0;
	for(i=0;i<8;i++)
	{
		ret <<= 5;
		if(ir_code & 0x01)
		ret |= 0x10;
		else
		ret |= 0x1e;      
		ir_code >>= 1;
	}
	return ret;
}

uint16_t atim_100us_count = 0;
void ir_docking_guide_send(void)
{	
	static uint64_t ir1 = 0;
	static uint64_t ir2 = 0;
	static uint64_t ir3 = 0;
	static uint64_t ir4 = 0;

	uint8_t a, b, c, d;

    atim_100us_count++;    
	if(atim_100us_count <= 280)
	{
		/* 在第一个周期初始化调制后的编码 */
		if(atim_100us_count == 1)
		{
			ir1 = ir_guide_code_modulate(0x48);
			ir2 = ir_guide_code_modulate(0x44); 
			ir3 = ir_guide_code_modulate(0x42);
			ir4 = ir_guide_code_modulate(0x50);
		}
		
		/* 读取当前最低位 */
		a = ir1 & 0x01;
		b = ir2 & 0x01;
		c = ir3 & 0x01;
		d = ir4 & 0x01;
		
		/* 根据编码位控制红外输出 */
		if(a > 0) 
			IR_LEFT_OFF;
		else 
			IR_LEFT_ON;
		
		if(b > 0) 
			IR_MIDDLE_LEFT_OFF;
		else 
			IR_MIDDLE_LEFT_ON;
		
		if(c > 0) 
			IR_MIDDLE_RIGHT_OFF;
		else 
			IR_MIDDLE_RIGHT_ON;
		
		if(d > 0) 
			IR_RIGHT_OFF;
		else 
			IR_RIGHT_ON;
		
		/* 每7个周期右移一位，发送下一位编码 */
		if((atim_100us_count % 7) == 0)
		{
			ir1 >>= 1;
			ir2 >>= 1;
			ir3 >>= 1;
			ir4 >>= 1;
		}
	}
	else if(atim_100us_count == 281)
	{
		/* 关闭所有红外输出 */
		IR_LEFT_OFF;
		IR_RIGHT_OFF;
		IR_MIDDLE_LEFT_OFF;
		IR_MIDDLE_RIGHT_OFF;
	}
	else if(atim_100us_count > 460)
	{
		/* 重置计数器，开始新一轮发送 */
		atim_100us_count = 0;
	}
}
