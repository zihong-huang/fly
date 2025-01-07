#ifndef __FUNCTION_TASK_H
#define __FUNCTION_TASK_H

#include "sys.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "Serial.h"
#include "bmp280.h"
#include "LED.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "nrf24l01.h"
#include "pid.h"
#include "flyconput.h"
#include "motor.h"
#include "power.h"
#include "imu.h"


typedef struct{
		float pitch;
		float roll;
		float yaw;
	
}imu_mpu;

extern imu_mpu imu;
//消息队列外部定义
extern xQueueHandle xQueue_BMP;
extern xQueueHandle xQueue_MPUX;
extern xQueueHandle xQueue_MPUY;
extern xQueueHandle xQueue_MPUZ;
extern xQueueHandle xQueue_POWER;
extern xQueueHandle xQueue_Altitude;

extern SemaphoreHandle_t SemapSTX_Handle_t;
extern SemaphoreHandle_t SemapSRX_Handle_t;

void Task_LED(void);
void Task_BMP(void *pvParameters);
int Task_MPU6050(void);
void Task_NRF_RX(void *pvParameters);
void Task_NRF_TX(void *pvParameters);
void Task_MOTOR(void);
void Cpu_task(void  * argument);
#endif
