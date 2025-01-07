#ifndef __MOTOR_H
#define __MOTOR_H

#include "sys.h"


// º¯ÊýÉùÃ÷
void PWM_Init(uint16_t arr, uint16_t psc);
void PWM_SetDutyCycle(uint16_t pulseA15, uint16_t pulseB3, uint16_t pulseB4, uint16_t pulseB5);
void Motor_Init(void);

#endif
