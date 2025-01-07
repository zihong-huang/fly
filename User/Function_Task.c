/************************************* Copyright(C)******************************
**
**                 (C) Copyright 2024, @mayuxin,China, GCU.
**                            All Rights Reserved
**                     				By( @myx)
**-------------------------------------------------------------------------------
** @FileName   : Function_Task.c
** @Brief	     :
**
**-------------------------------------------------------------------------------
** @Author     : @mayuxin
** @Version    : v1.0
** @Date       : 2024-10-08
**-------------------------------------------------------------------------------
** @Mender	   : Node
** @Version    : Node
** @Date       : Node
** @Brief      : Node
********************************************************************************/
#include "Function_Task.h"


//创建发送消息队列
xQueueHandle xQueue_BMP;
xQueueHandle xQueue_MPUX;
xQueueHandle xQueue_MPUY;
xQueueHandle xQueue_MPUZ;
xQueueHandle xQueue_POWER;
xQueueHandle xQueue_Altitude;

//创建二值信号量
SemaphoreHandle_t SemapSTX_Handle_t;//发送任务二值信号量句柄
SemaphoreHandle_t SemapSRX_Handle_t;//接收任务二值信号量句柄
imu_mpu imu;


/****************************** BEFIN ********************************
**
**@Name       : Task_LED
**@Brief      : None
**@Param : [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-15
******************************** END *********************************/

void Task_LED()
{
	
	while(1)
	{
		
		LED_OFF();
		vTaskDelay(pdMS_TO_TICKS(10));
		
		LED_ON();
		vTaskDelay(pdMS_TO_TICKS(10));
		
		
		vTaskDelay(pdMS_TO_TICKS(5));
		
	}
}




/****************************** BEFIN ********************************
**
**@Name       : Task_BMP
**@Brief      : None
**@Param : [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-15
******************************** END *********************************/

void Task_BMP(void *pvParameters)
{
	
//		BaseType_t status_Power;
		
		double BMP_Pressure;
		double BMP_Temperature, altitude;
		char Power[8];
		u8 Battery[4];
		float voltage;

		
		while(1)
		{
				while(BMP280_GetStatus(BMP280_MEASURING) != RESET);
				while(BMP280_GetStatus(BMP280_IM_UPDATE) != RESET);
				BMP_Pressure = BMP280_GetIIR_Pressure();
				BMP_Temperature = BMP280_GetIIR_Temperature();
				altitude = BMP280_CalculateAltitude(BMP_Pressure, 101325);
				voltage = ADC_Get_Voltage();
				
		//		taskENTER_CRITICAL();
				printf("\r\ntem:%.2f, altitute:%.2f\r\n",BMP_Temperature, altitude);
				printf("\r\npower=%d \tvol:%.2f\r\n",ADC_Get_Value(),voltage);
		//		taskEXIT_CRITICAL();
//				printf("\r\ndelay up\r\n");
				vTaskDelay(pdMS_TO_TICKS(10));
//				printf("\r\ndelay bottom\r\n");
				//写入传输数据长度
				Power[0] = 7;
				//写入帧头
				Power[1] = 0xFC;
				Power[2] = 0xFA;
				//进制转换
				floatToHexArray(voltage, Battery);
				Power[3] = Battery[0];
				Power[4] = Battery[1];
				Power[5] = Battery[2];
				Power[6] = Battery[3];
				//写入帧尾
				Power[7] = 0xFE;
				//写入队列
//				printf("\r\rqu\r\n");
//				status_Power = xQueueSend(xQueue_POWER, (void*)Power, portMAX_DELAY);
//				if(status_Power !=pdPASS)
//				{
//						printf("\r\nerror\r\n");
//				}		
//				printf("\r\nqd\r\n");
		}
	
}





/****************************** BEFIN ********************************
**
**@Name       : Task_MPU6050
**@Brief      : None
**@Param : [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-10-08
******************************** END *********************************/

int Task_MPU6050()
{

		
		while(1)
		{
				
				

				MPU_Get_data();
				mpu_dmp_get_data(&imu.pitch, &imu.roll, &imu.yaw);
				APP_Fligh_PID_Control();
				APP_Flight_Motor_Control();
			
//				taskENTER_CRITICAL();
//				printf("\r\n pitch:%.1f\t roll:%.1f\t yaw:%.1f \r\n", imu.pitch,imu.roll, imu.yaw);
//				printf("\r\n GyroX:%d\t GyroY:%d\t GryoZ:%d\r\n", MPU6050.gyroX, MPU6050.gyroY,MPU6050.gyroZ);
////				printf("\r\n pitch:%.1f\t roll:%.1f\t yaw:%.1f \r\n", Angle.pitch,Angle.roll, Angle.yaw);
//				taskEXIT_CRITICAL();
//				delay_ms(6);
				vTaskDelay(pdMS_TO_TICKS(20));

				
				
		}
	
}



/****************************** BEFIN ********************************
**
**@Name       : Task_NRF_TX
**@Brief      : None
**@Param : [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-10-08
******************************** END *********************************/

void Task_NRF_TX(void *pvParameters)
{

		char Receive_Power[8];
		BaseType_t Power;
		
		while(1)
		{
//				while(xSemaphoreTake(SemapSTX_Handle_t, portMAX_DELAY))
//				{
						TX_Mode();
						//从队列读消息
						vTaskDelay(pdMS_TO_TICKS(20)); 
						
//						u8 exp[8]={7, 0xFA, 0xFB, 0x07, 0xd4, 0x00, 0x00, 0xFE};
//						if(NRF24L01_TxPacket((u8*)exp) == TX_OK)
//						printf("\r\nsend is succeed\r\n");
//						else printf("\r\nsend id fail\r\n");
						
						Power = xQueueReceive(xQueue_POWER, (void*)Receive_Power, portMAX_DELAY);
						if(Power != pdTRUE)
						{
								printf("\r\nerror\r\n");
						}
						if(NRF24L01_TxPacket((u8*)Receive_Power) == TX_OK)
						printf("\r\nsend is succeed\r\n");
						else printf("\r\nsend id fail\r\n");
									
						vTaskDelay(pdMS_TO_TICKS(10));
//						xSemaphoreGive(SemapSRX_Handle_t);
//						
//				}
//				vTaskDelay(pdMS_TO_TICKS(10));
		}
	

}




/****************************** BEFIN ********************************
**
**@Name       : Task_MOTOR
**@Brief      : None
**@Param None 
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-15
******************************** END *********************************/

void Task_MOTOR(void)
{
	
		while(1)
		{
				PWM_SetDutyCycle(RX_DVR.thorttle-1000, RX_DVR.thorttle-1000, RX_DVR.thorttle-1000, RX_DVR.thorttle-1000);
			  vTaskDelay(pdMS_TO_TICKS(10));
				//printf("MOTOR\r\n");
		}
}




/****************************** BEFIN ********************************
**
**@Name       : Task_NRF_RX
**@Brief      : None
**@Param pvParameters: [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-21
******************************** END *********************************/

void Task_NRF_RX(void *pvParameters)
{
//		u8 i;
		char arr[11];
		char rx_arr[10];
    while (1)
    {
			
        // 等待接收信号量
//        if (xSemaphoreTake(SemapSRX_Handle_t, portMAX_DELAY) == pdTRUE)
//        {
            // 设置接收模式
            RX_Mode();
            vTaskDelay(pdMS_TO_TICKS(20)); // 等待 NRF24L01 模块进入 RX 模式

            uint8_t rx_status = NRF24L01_RxPacket((u8 *)arr);
            if (rx_status == 0) // 数据接收成功
            {
                printf("\r\nRX_succeed\r\n");

                // 提取有效数据（跳过头部）

                memcpy(rx_arr, &arr[1], 10);

                // 数据分析
                Analyze_data(rx_arr);
            }
            else // 接收失败
            {
                printf("\r\nRX_failed\r\n");
            }

            // 短暂延时，避免过快切换
            vTaskDelay(pdMS_TO_TICKS(10));

            // 释放发送信号量
//            xSemaphoreGive(SemapSTX_Handle_t);
//						
//        }
//				vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/****************************** BEFIN ********************************
**
**@Name       : Cpu_task
**@Brief      : None
**@Param argument: [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-20
******************************** END *********************************/

void Cpu_task(void  * argument)
{
  /* USER CODE BEGIN Cpu_task */
    uint8_t CPU_RunInfo[512];
  /* Infinite loop */
  while(1)
  {
      memset(CPU_RunInfo,0,512);
      vTaskList((char *)&CPU_RunInfo); //获取任务运行时间信息
			taskENTER_CRITICAL();
      printf("---------------------------------------------\r\n");
      printf("任务名       任务状态   优先级  剩余栈  任务序号\r\n");
      printf("%s", CPU_RunInfo);
      printf("---------------------------------------------\r\n");
			taskEXIT_CRITICAL();
      vTaskDelay(5000); /* 延时500个tick */
  }
  /* USER CODE END Cpu_task */
}


