#ifndef __POWER_H
#define __POWER_H

#include "sys.h"

// ��ʼ��ADC
void ADC_Init_Config(void);

// ��ȡADC��ת��ֵ
uint16_t ADC_Get_Value(void);

// ��ȡPB1�ĵ�ѹֵ����λ�����أ�
float ADC_Get_Voltage(void);

// ��ȡ��ص����ٷֱ�
uint8_t Get_Battery_Percentage(void);



#endif 
