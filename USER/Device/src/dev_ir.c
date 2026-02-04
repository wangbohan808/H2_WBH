#include "dev_ir.h"


void ir_open_all(void)
{
    IR_LEFT_ON;
    IR_RIGHT_ON;
    IR_MIDDLE_LEFT_ON;
    IR_MIDDLE_RIGHT_ON;
}

void ir_close_all(void)
{
    IR_LEFT_OFF;
    IR_RIGHT_OFF;
    IR_MIDDLE_LEFT_OFF;
    IR_MIDDLE_RIGHT_OFF;
}

