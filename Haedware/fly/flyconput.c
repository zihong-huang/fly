#include "flyconput.h"

PidProject pidPitch;
PidProject pidRoll;
PidProject pidYaw;
PidProject pidRateX;
PidProject pidRateY;
PidProject pidRateZ;

PidProject *pids[] = {&pidPitch, &pidRoll, &pidYaw, &pidRateX, &pidRateY, &pidRateZ};


DVR RX_DVR;
Rx_Key_state Rx_Key;

u8 unlock = 0;


int motor1 = 0;		//左前
int motor2 = 0;		//右前
int motor3 = 0;		//左后
int motor4 = 0;		//右后

//void Get_posture(void)
//{
//		mpu_dmp_get_data(&Angle.pitch, &Angle.roll, &Angle.yaw);
//		MPU_Get_Gyroscope(&MPU6050.gyroX, &MPU6050.gyroY, &MPU6050.gyroZ);
//}




/****************************** BEFIN ********************************
**
**@Name       : APP_Fligh_PID_Control
**@Brief      : None
**@Param : [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-16
******************************** END *********************************/

void APP_Fligh_PID_Control(void)
{
		static u8 status = WAITTING1;
		switch(status)
		{
			case WAITTING1:
						if(unlock == 1)
						{
								status = WAITTING2;
						}
						break;
			case WAITTING2:
						ResetPID(pids, 6);
						status = PROCESS;
						break;
			case PROCESS:
						//赋值角度的测量值
						pidPitch.measured = imu.pitch;
						pidRoll.measured  = imu.roll;
						pidYaw.measured		= imu.yaw;
						//赋值角速度测量值
						pidRateX.measured = MPU6050.gyroX;
						pidRateY.measured = MPU6050.gyroY;
						pidRateZ.measured = MPU6050.gyroZ;
						/*
								俯仰角->y轴角速度
								横滚角->x轴角速度
								偏航角->z轴角速度
						*/
						CasecadePID(&pidPitch, &pidRateX, 0.002f);
						CasecadePID(&pidRoll, &pidRateY, 0.002f);
						CasecadePID(&pidYaw, &pidRateZ, 0.002f);
			
						break;
			default:
						break;
			
		}
}

/*

电机 右下4 转速 = 油门+横滚 PID 输出 - 俯仰 PID 输出 - 偏航PID 输出
电机 右上2 转速 = 油门+横滚 PID 输出 + 俯仰 PID 输出 + 偏航PID 输出
电机 左上1 转速 = 油门-横滚 PID 输出 + 俯仰 PID 输出 - 偏航PID 输出
电机 左下3 转速 = 油门-横滚 PID 输出 - 俯仰 PID 输出 + 偏航PID 输出

横滚 PID 输出 = 外环 横滚角 -> 内环X轴 角速度
俯仰 PID 输出 = 外环 俯仰角 -> 内环Y轴 角速度
偏航 PID 输出 = 外环 偏航角 -> 内环Z轴 角速度

*/

/****************************** BEFIN ********************************
**
**@Name       : APP_Flight_Motor_Control
**@Brief      : None
**@Param : [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-16
******************************** END *********************************/

void APP_Flight_Motor_Control(void)
{
		
		
		static u8 status;
		if(Rx_Key.key7_flasg == 1)
		{
				status = WAITTING1;
//				Rx_Key.key7_flasg = 0;
				LED_State_Choice(2);
		}
		switch(status)
		{
			//判断解锁然后进入阶段二
			case WAITTING1:
						motor1 = motor2 = motor3 = motor4 = 0;
						status = WAITTING2;
						break;
			//判断油门动了 > 1100进入控制
			case WAITTING2:
						if(RX_DVR.thorttle > 1100)
						{
								status = PROCESS;
						}
			
						break;
			//正式控制： 摇杆-1000 if 油门<1050 pwm = 0, 限制900 +3PID值
			case PROCESS:
						//摇杆值转成PWM范围值 - 1000
						motor1 = RX_DVR.thorttle - 1000;
						motor2 = RX_DVR.DvrX - 1000;
						motor3 = RX_DVR.DvrY - 1000;
						motor4 = 500;
						
						if(RX_DVR.thorttle <	1050)
						{
								motor1 = motor2 = motor3 = motor4 = 0;
						}
						
						clamp(&motor1, 0, 900);
						clamp(&motor2, 0, 900);
						clamp(&motor3, 0, 900);
						clamp(&motor4, 0, 900);
						
						motor1 +=	-pidRateX.out - pidRateY.out - pidRateZ.out;	//右前
						motor2 +=	+pidRateX.out - pidRateY.out + pidRateZ.out;	//左前
						motor3 += -pidRateX.out + pidRateY.out + pidRateZ.out;	//右后
						motor4 +=	+pidRateX.out + pidRateY.out - pidRateZ.out;	//左后
						
						clamp(&motor1, 0, 1000);
						clamp(&motor2, 0, 1000);
						clamp(&motor3, 0, 1000);
						clamp(&motor4, 0, 1000);
						
						break;
			default:
						break;
			
		}
		
		PWM_SetDutyCycle( motor4, motor2, motor3, motor1);
}

void App_PID_Param_Init(void)
{
		//内环	
		pidRateX.kp = 0.0f;
		pidRateY.kp = 0.0f;
		pidRateZ.kp = 0.0f;

		pidRateX.ki = 0.0f;
		pidRateY.ki = 0.0f;
		pidRateZ.ki = 0.0f;

		pidRateX.kd = 0.0f;
		pidRateY.kd = 0.0f;
		pidRateZ.kd = 0.0f;
		
		//外环
		pidPitch.kp = 0.0f;
		pidRoll.kp  = 0.0f;
		pidYaw.kp   = 0.0f;
	
		pidPitch.ki = 0.0f;
		pidRoll.ki  = 0.0f;
		pidYaw.ki   = 0.0f;
		
		pidPitch.kd = 0.0f;
		pidRoll.kd  = 0.0f;
		pidYaw.kd   = 0.0f;
	
	
	
}


/****************************** BEFIN ********************************
**
**@Name       : Analyze_data
**@Brief      : None
**@Param Analyze_arr[]: [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-16
******************************** END *********************************/

void Analyze_data(char Analyze_arr[])
{
		Rx_Key.key7_flasg = 0;
		char Tho_analyze[2];	
		char DVRX_analyze[2];	
		char DVRY_analyze[2];	
		char KEY_analyze[2];	
		
		//帧头判断,检查帧尾
		if(Analyze_arr[0] == 0xFD && Analyze_arr[1] == 0xFD && Analyze_arr[9] == 0xFE)
		{
				/* THR */
				Tho_analyze[0] = Analyze_arr[2];  // 高位字节
				Tho_analyze[1] = Analyze_arr[3];
				//写入THR数据
				RX_DVR.thorttle = (Tho_analyze[0] << 8) | (Tho_analyze[1]);
				/* PIT */
				DVRX_analyze[0] = Analyze_arr[4];  // 高位字节
				DVRX_analyze[1] = Analyze_arr[5];
				//写入DVRX数据
				RX_DVR.DvrX = (DVRX_analyze[0] << 8) | (DVRX_analyze[1]);
				/* ROL */
				DVRY_analyze[0] = Analyze_arr[6];  // 高位字节
				DVRY_analyze[1] = Analyze_arr[7];
				//写入DVRY数据
				RX_DVR.DvrY = (DVRY_analyze[0] << 8) | (DVRY_analyze[1]);
				/* KEY */
				// 提取数据并存储到新的数组中
				KEY_analyze[0] = Analyze_arr[8];  // 高位字节
				if(KEY_analyze[0] == 0x06)
				Rx_Key.key7_flasg = 1;
				

				printf("receive_sucessful\r\n");
				printf("\r\n thr:%d, pit:%d. rol:%d \r\n",RX_DVR.thorttle,RX_DVR.DvrX, RX_DVR.DvrY);

				
		}
		else
		{
				memset(Tho_analyze, 0, sizeof(Tho_analyze));
				memset(DVRX_analyze, 0, sizeof(DVRX_analyze));
				memset(DVRY_analyze, 0, sizeof(DVRY_analyze));
				memset(KEY_analyze, 0, sizeof(KEY_analyze));
				printf("Check no FH");
		}
		
}


/*
		将浮点型转成16进制数组
*/

/****************************** BEFIN ********************************
**
**@Name       : floatToHexArray
**@Brief      : None
**@Param num: [输入/出] 
**			 hexArray[4]: [输入/出]  
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-11-16
******************************** END *********************************/

void floatToHexArray(float num, uint8_t hexArray[4]) 
{
    union {
        float f;
        uint32_t i;
    } floatUnion;

    floatUnion.f = num;

    // 将 32 位整数的每个字节存储到数组中
    hexArray[0] = (floatUnion.i >> 24) & 0xFF; // 最高字节
    hexArray[1] = (floatUnion.i >> 16) & 0xFF;
    hexArray[2] = (floatUnion.i >> 8) & 0xFF;
    hexArray[3] = floatUnion.i & 0xFF;         // 最低字节
}

void clamp(int *target, int min_val, int max_val) 
{
    if (*target < min_val) {
        *target = min_val;  // 将目标值设置为最小值
    } else if (*target > max_val) {
        *target = max_val;  // 将目标值设置为最大值
    }
}
