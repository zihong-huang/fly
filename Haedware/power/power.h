#ifndef __POWER_H
#define __POWER_H

#include "sys.h"

// 初始化ADC
void ADC_Init_Config(void);

// 获取ADC的转换值
uint16_t ADC_Get_Value(void);

// 获取PB1的电压值（单位：伏特）
float ADC_Get_Voltage(void);

// 获取电池电量百分比
uint8_t Get_Battery_Percentage(void);



#endif 
