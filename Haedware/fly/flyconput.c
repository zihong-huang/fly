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


int motor1 = 0;		//��ǰ
int motor2 = 0;		//��ǰ
int motor3 = 0;		//���
int motor4 = 0;		//�Һ�

//void Get_posture(void)
//{
//		mpu_dmp_get_data(&Angle.pitch, &Angle.roll, &Angle.yaw);
//		MPU_Get_Gyroscope(&MPU6050.gyroX, &MPU6050.gyroY, &MPU6050.gyroZ);
//}




/****************************** BEFIN ********************************
**
**@Name       : APP_Fligh_PID_Control
**@Brief      : None
**@Param : [����/��]  
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
						//��ֵ�ǶȵĲ���ֵ
						pidPitch.measured = imu.pitch;
						pidRoll.measured  = imu.roll;
						pidYaw.measured		= imu.yaw;
						//��ֵ���ٶȲ���ֵ
						pidRateX.measured = MPU6050.gyroX;
						pidRateY.measured = MPU6050.gyroY;
						pidRateZ.measured = MPU6050.gyroZ;
						/*
								������->y����ٶ�
								�����->x����ٶ�
								ƫ����->z����ٶ�
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

��� ����4 ת�� = ����+��� PID ��� - ���� PID ��� - ƫ��PID ���
��� ����2 ת�� = ����+��� PID ��� + ���� PID ��� + ƫ��PID ���
��� ����1 ת�� = ����-��� PID ��� + ���� PID ��� - ƫ��PID ���
��� ����3 ת�� = ����-��� PID ��� - ���� PID ��� + ƫ��PID ���

��� PID ��� = �⻷ ����� -> �ڻ�X�� ���ٶ�
���� PID ��� = �⻷ ������ -> �ڻ�Y�� ���ٶ�
ƫ�� PID ��� = �⻷ ƫ���� -> �ڻ�Z�� ���ٶ�

*/

/****************************** BEFIN ********************************
**
**@Name       : APP_Flight_Motor_Control
**@Brief      : None
**@Param : [����/��]  
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
			//�жϽ���Ȼ�����׶ζ�
			case WAITTING1:
						motor1 = motor2 = motor3 = motor4 = 0;
						status = WAITTING2;
						break;
			//�ж����Ŷ��� > 1100�������
			case WAITTING2:
						if(RX_DVR.thorttle > 1100)
						{
								status = PROCESS;
						}
			
						break;
			//��ʽ���ƣ� ҡ��-1000 if ����<1050 pwm = 0, ����900 +3PIDֵ
			case PROCESS:
						//ҡ��ֵת��PWM��Χֵ - 1000
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
						
						motor1 +=	-pidRateX.out - pidRateY.out - pidRateZ.out;	//��ǰ
						motor2 +=	+pidRateX.out - pidRateY.out + pidRateZ.out;	//��ǰ
						motor3 += -pidRateX.out + pidRateY.out + pidRateZ.out;	//�Һ�
						motor4 +=	+pidRateX.out + pidRateY.out - pidRateZ.out;	//���
						
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
		//�ڻ�	
		pidRateX.kp = 0.0f;
		pidRateY.kp = 0.0f;
		pidRateZ.kp = 0.0f;

		pidRateX.ki = 0.0f;
		pidRateY.ki = 0.0f;
		pidRateZ.ki = 0.0f;

		pidRateX.kd = 0.0f;
		pidRateY.kd = 0.0f;
		pidRateZ.kd = 0.0f;
		
		//�⻷
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
**@Param Analyze_arr[]: [����/��]  
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
		
		//֡ͷ�ж�,���֡β
		if(Analyze_arr[0] == 0xFD && Analyze_arr[1] == 0xFD && Analyze_arr[9] == 0xFE)
		{
				/* THR */
				Tho_analyze[0] = Analyze_arr[2];  // ��λ�ֽ�
				Tho_analyze[1] = Analyze_arr[3];
				//д��THR����
				RX_DVR.thorttle = (Tho_analyze[0] << 8) | (Tho_analyze[1]);
				/* PIT */
				DVRX_analyze[0] = Analyze_arr[4];  // ��λ�ֽ�
				DVRX_analyze[1] = Analyze_arr[5];
				//д��DVRX����
				RX_DVR.DvrX = (DVRX_analyze[0] << 8) | (DVRX_analyze[1]);
				/* ROL */
				DVRY_analyze[0] = Analyze_arr[6];  // ��λ�ֽ�
				DVRY_analyze[1] = Analyze_arr[7];
				//д��DVRY����
				RX_DVR.DvrY = (DVRY_analyze[0] << 8) | (DVRY_analyze[1]);
				/* KEY */
				// ��ȡ���ݲ��洢���µ�������
				KEY_analyze[0] = Analyze_arr[8];  // ��λ�ֽ�
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
		��������ת��16��������
*/

/****************************** BEFIN ********************************
**
**@Name       : floatToHexArray
**@Brief      : None
**@Param num: [����/��] 
**			 hexArray[4]: [����/��]  
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

    // �� 32 λ������ÿ���ֽڴ洢��������
    hexArray[0] = (floatUnion.i >> 24) & 0xFF; // ����ֽ�
    hexArray[1] = (floatUnion.i >> 16) & 0xFF;
    hexArray[2] = (floatUnion.i >> 8) & 0xFF;
    hexArray[3] = floatUnion.i & 0xFF;         // ����ֽ�
}

void clamp(int *target, int min_val, int max_val) 
{
    if (*target < min_val) {
        *target = min_val;  // ��Ŀ��ֵ����Ϊ��Сֵ
    } else if (*target > max_val) {
        *target = max_val;  // ��Ŀ��ֵ����Ϊ���ֵ
    }
}
