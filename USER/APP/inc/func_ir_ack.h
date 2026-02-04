#ifndef __FUNC_IR_ACK_H
#define __FUNC_IR_ACK_H

#include <stdint.h>


void ir_send_ack(void);
void set_ir_send_count(uint8_t count);
void set_ir_sent_bite(uint8_t count);
extern uint8_t sent_buff[13];
extern char version[2];
extern uint8_t soft_version;

#endif /* __FUNC_IR_ACK_H */

