#ifndef __DELAY_H
#define __DELAY_H
#include "sys.h"

#define OS	1			//����ϵͳ���ã�1�� �в���ϵͳ��0�� û�в���ϵͳ

void delay_init(void);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

#endif
