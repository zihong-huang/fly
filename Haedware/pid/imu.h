#ifndef __IMU_H
#define __IMU_H

#include "sys.h"
#include "mpu6050.h"
#define DEG2RAD 0.017453293f 
#define RAD2DEG 57.29578f
#define squa(Sq) (((float)Sq)*((float)Sq))

typedef struct
{
		float yaw;
		float pitch;
		float roll;
		float yaw_mag;
		float Cos_Roll;
		float Sin_Roll;
		float Cos_Pitch;
		float Sin_Pitch;
		float Cos_Yaw;
		float Sin_Yaw;
}_stIMU;


typedef struct V
{
		float x;
		float y;
		float z;
}MPUDA;

extern float yaw_control;
extern float Yaw_correct;
extern _stIMU IMU;
extern float GetAccz(void);
extern void GetAngle(const _stMPU *pMPU, _stAngle *pAngle, float dt );
extern MPUDA Gravity, Acc, Gyro, AccGravity;

#endif


