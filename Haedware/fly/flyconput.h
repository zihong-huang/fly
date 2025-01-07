#ifndef __FLY_H
#define __FLY_H

#include "sys.h"
#include "pid.h"
#include "inv_mpu.h"
#include "MPU6050.h"
#include "Function_Task.h"

#define WAITTING1 0
#define WAITTING2 1
#define PROCESS 2


#define rx_dvr_pwm4 4

typedef struct{
		u16 thorttle;
		u16 DvrX;
		u16 DvrY;
}DVR;

typedef struct Key_state{
	u8 key2_flasg;
	u8 key3_flasg;
	u8 key4_flasg;
	u8 key5_flasg;
	u8 key6_flasg;
	u8 key7_flasg;
	u8 key8_flasg;
	u8 key9_flasg;
	u8 key10_flasg;
	u8 sw1_flasg;
	u8 sw2_flasg;
}Rx_Key_state;

typedef struct{
		float pitch;
		float roll;
		float yaw;
}mpu_data;


extern DVR RX_DVR;

void floatToHexArray(float num, uint8_t hexArray[4]);


void Get_posture(void);

void APP_Fligh_PID_Control(void);
void APP_Flight_Motor_Control(void);
void App_PID_Param_Init(void);

void Analyze_data(char Analyze_arr[]);
void clamp(int *target, int min_val, int max_val);

extern u8 unlock;

#endif
