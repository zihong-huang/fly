#include<stdio.h>
#include "sys.h"

#include "Function_Task.h"

#include "Serial.h"
#include "LED.h"
#include "nrf24l01.h"
#include "bmp280.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "power.h"
#include "motor.h"
#include "flyconput.h"
#include "imu.h"
////��̬����LED
//#define START_TASK_LED  1							//�������ȼ�
//#define START_STK_SIZE	64						//��ջ��С
//TaskHandle_t	Task_LED_Handle;				//������


//��̬����BMP280
#define START_TASK_BMP280 2						//�������ȼ�
#define START_BMP_SIZE		256					//��ջ��С
TaskHandle_t	Task_BMP280_Handle;			//������
	
//��̬����MPU6050
#define START_TASK_MPU6050 2					//�������ȼ�
#define START_MPU_SIZE		128					//��ջ��С
TaskHandle_t	Task_MPU6050_Handle;		//������


//��̬����NRF_TX
#define START_TASK_NRF_TX 2						//�������ȼ�
#define START_NRF_TX_SIZE		256				//��ջ��С
TaskHandle_t	Task_NRF_TX_Handle;			//������

//��̬����NRF_RX
#define START_TASK_NRF_RX 2						//�������ȼ�
#define START_NRF_RX_SIZE		256				//��ջ��С
TaskHandle_t	Task_NRF_RX_Handle;			//������

//��̬����MOTOR
#define START_TASK_MOTOR  2						//�������ȼ�
#define START_MOTOR_SIZE		128				//��ջ��С
TaskHandle_t	Task_MOTOR_Handle;			//������

//��̬����CPU
#define START_TASK_CPU  2							//�������ȼ�
#define START_CPU_SIZE		256					//��ջ��С
TaskHandle_t	Task_CPU_Handle;				//������

int main(void)
{
	
	RX_DVR.thorttle = 1000;
	u8 flag, nrf_flag;
	delay_init();
	LED_Init();
	ADC_Init_Config();
	Serial_Init(115200);
	Bmp_Init();
	flag = MPU_Init();
	Get_MPU_Offset();
	mpu_dmp_init();
	Init_NRF24L01();
	printf("ID:%d\r\n", flag);
	nrf_flag = NRF24L01_Check();
	if(nrf_flag) printf("NO:%d\r\n", nrf_flag);
	else printf("\r\nYes:%d\r\n", nrf_flag);
	PWM_Init(1000-1, 72-1);
	Motor_Init();
	
	LED_State_Choice(4);
//	PWM_SetDutyCycle(150, 150, 150, 150);
	

	
//	xQueue_POWER = xQueueCreate(2, sizeof(char[8]));
//	if (xQueue_POWER == NULL)printf("Queue creation failed!\n");
//	else printf("\r\ncreate succeeful\r\n");
	
//	SemapSTX_Handle_t = xSemaphoreCreateBinary();
//	SemapSRX_Handle_t = xSemaphoreCreateBinary();
//	xSemaphoreGive(SemapSRX_Handle_t);
	
//	xTaskCreate((TaskFunction_t)	Task_LED,					//����LED�߳�
//						(char *)					"Task_LED",
//						(uint16_t)				START_STK_SIZE,
//						(void *) 					NULL,
//						(UBaseType_t)			START_TASK_LED,
//						(TaskHandle_t *)	&Task_LED_Handle );
	
	
//	xTaskCreate((TaskFunction_t)	Task_BMP,					//����BMP�߳�
//						(char *)					"Task_BMP280",
//						(uint16_t)				START_BMP_SIZE,
//						(void *) 					NULL,
//						(UBaseType_t)			START_TASK_BMP280,
//						(TaskHandle_t *)	&Task_BMP280_Handle );		


	xTaskCreate((TaskFunction_t)	Task_MPU6050,					//����MPU�߳�
						(char *)					"Task_MPU6050",
						(uint16_t)				START_MPU_SIZE,
						(void *) 					NULL,
						(UBaseType_t)			START_TASK_MPU6050,
						(TaskHandle_t *)	&Task_MPU6050_Handle );		

//	xTaskCreate((TaskFunction_t)	Task_NRF_TX,					//����NRF�߳�
//						(char *)					"Task_NRF_TX",
//						(uint16_t)				START_NRF_TX_SIZE,
//						(void *) 					NULL,
//						(UBaseType_t)			START_TASK_NRF_TX,
//						(TaskHandle_t *)	&Task_NRF_TX_Handle );
						
	xTaskCreate((TaskFunction_t)	Task_NRF_RX,					//����NRF�߳�
						(char *)					"Task_NRF_RX",
						(uint16_t)				START_NRF_RX_SIZE,
						(void *) 					NULL,
						(UBaseType_t)			START_TASK_NRF_RX,
						(TaskHandle_t *)	&Task_NRF_RX_Handle );

	xTaskCreate((TaskFunction_t)	Task_MOTOR,					//����motor�߳�
						(char *)					"Task_MOTOR",
						(uint16_t)				START_MOTOR_SIZE,
						(void *) 					NULL,
						(UBaseType_t)			START_TASK_MOTOR,
						(TaskHandle_t *)	&Task_MOTOR_Handle );
					
//	xTaskCreate((TaskFunction_t)	Cpu_task,					//����CPU�߳�
//						(char *)					"Task_Cpu",
//						(uint16_t)				START_CPU_SIZE,
//						(void *) 					NULL,
//						(UBaseType_t)			START_TASK_CPU,
//						(TaskHandle_t *)	&Task_CPU_Handle );		

						
						
	
	vTaskStartScheduler();
	
	
	
	
//	float pitch, roll, yaw;
//	double BMP_Pressure;
//	double BMP_Temperature, altitute;
//	u8 Arr[32];
//	Arr[0] = 1;
//	Arr[1] = 2;

//	u8 RX_Arr[8];
//	char Check[7];
//	RX_Mode();
	

//		u16 led0pwmval=0;    
//		u8 dir=1;

	while(1)
	{
			
//			if(NRF24L01_TxPacket(Arr) == TX_OK)
//			{
//				printf("TX_succeed\r\n");
//			}
//			else printf("TX_error\r\n");
		
//				if(NRF24L01_RxPacket(RX_Arr) == 0)
//				{
//					u8 i;
//					printf("RX_succeed\r\n");
//					for(i = 0; i<8; i++)
//					{
//							Check[i] = RX_Arr[i+1];
//						//printf("RX:%s", Check);
//					}
//					
//					
////					printf("RX:%.2f\r\n", Byte_to_Float(RX_Arr));
//					// ���֡ͷ
//					Analyze_data(Check);
//					
//					
//					
//				 
//				}
//				else printf("RX_error\r\n");
			
		

//			BMP_Temperature = BMP280_GetIIR_Temperature();
//			BMP_Pressure = BMP280_GetIIR_Pressure();
//			altitute = BMP280_CalculateAltitude(BMP_Pressure, 101325);
//			printf("\r\nPressure:%.2f\r\n",BMP_Pressure);
//			delay_ms(100);
//			printf("tem:%.2f, altitute:%.2f\r\n",BMP_Temperature, altitute);
//			delay_ms(100);
		
		
//			if(dir)led0pwmval++;//dir==1 led0pwmval����
//			else led0pwmval--;	//dir==0 led0pwmval�ݼ� 
//			if(led0pwmval>=100)dir=0;//led0pwmval����300�󣬷���Ϊ�ݼ�
//			if(led0pwmval==0)dir=1;	//led0pwmval�ݼ���0�󣬷����Ϊ����

//			PWM_SetDutyCycle(led0pwmval);
		
//			float adc_voltage = ADC_Get_Voltage();
//			u8 Battery = Get_Battery_Percentage();
//			printf("DC:%.2f\r\n",adc_voltage);
//			printf("DCD:%d\r\n", Battery);
//			

//			mpu_dmp_get_data(&pitch, &roll, &yaw);
//			printf("pitch:%.2f, roll:%.2f, yaw:%.2f\r\n",pitch, roll, yaw);
//			printf("pitch_Hex:%x\r\n",floatPtrToHex(&pitch));
			
			
			
//			LED_OFF();
//			delay_ms(10);
//			LED_ON();
//			delay_ms(10);
	}
}



