#ifndef __DELAY_H
#define __DELAY_H
#include "sys.h"

#define OS	1			//操作系统设置：1， 有操作系统；0， 没有操作系统

void delay_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

#endif
