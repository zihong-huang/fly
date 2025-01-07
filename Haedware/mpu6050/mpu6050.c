#include "mpu6050.h"
#include "kalman.h"
_stMPU MPU6050;
_stAngle Angle;

u16 mpu_Offset[6] = {0};

u8 MPU_Init(void)
{ 
	u8 res;
//  GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);//ʹ��AFIOʱ�� 
/*
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTAʱ�� 
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	 // �˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
  GPIO_Init(GPIOA, &GPIO_InitStructure);					 //�����趨������ʼ��GPIOA

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//��ֹJTAG,�Ӷ�PA15��������ͨIOʹ��,����PA15��������ͨIO!!!
	
	MPU_AD0_CTRL=0;			//����MPU6050��AD0��Ϊ�͵�ƽ,�ӻ���ַΪ:0X68
*/
	MPU_IIC_Init();//��ʼ��IIC����
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//��λMPU6050
  delay_ms(100);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//����MPU6050 
	MPU_Set_Gyro_Fsr(3);					//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					//���ٶȴ�����,��2g
	MPU_Set_Rate(50);						//���ò�����50Hz
	MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//�ر������ж�
	MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT���ŵ͵�ƽ��Ч
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//����ID��ȷ
	{
		printf("MPUID:%x",res);
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//����CLKSEL,PLL X��Ϊ�ο�
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//���ٶ��������Ƕ�����
		MPU_Set_Rate(100);						//���ò�����Ϊ50Hz
 	}else return 1;
	return 0;
}
//����MPU6050�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);//���������������̷�Χ  
}
//����MPU6050���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}
//����MPU6050�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}
//����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//�õ�������ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

//�õ����ٶ�ֵ(ԭʼֵ)
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;;
}

/****************************** BEFIN ********************************
**
**@Name       : MPU_Get_data
**@Brief      : �����ȡԭʼ����
**@Param None 
**@Return     : None
**@Author     : @mayuxin
**@Data	      : 2024-12-08
******************************** END *********************************/

void MPU_Get_data(void)
{
		//1.��ȡԭʼ����
		MPU_Get_Accelerometer(&MPU6050.accX,&MPU6050.accY,&MPU6050.accZ);
		MPU_Get_Gyroscope(&MPU6050.gyroX,&MPU6050.gyroY,&MPU6050.gyroZ);
	
		//"0"ƫУ׼
		MPU6050.accX = MPU6050.accX - mpu_Offset[0];
		MPU6050.accY = MPU6050.accY - mpu_Offset[1];
		MPU6050.accZ = MPU6050.accZ - mpu_Offset[2];
		MPU6050.gyroX = MPU6050.gyroX - mpu_Offset[3];
		MPU6050.gyroY = MPU6050.gyroY - mpu_Offset[4];
		MPU6050.gyroZ = MPU6050.gyroZ - mpu_Offset[5];
	
	
		//2.�Լ��ٶȽ��п������˲�
		Com_Kalman_1(&ekf[0], MPU6050.accX);
		MPU6050.accX = (int16_t)ekf[0].out;
		Com_Kalman_1(&ekf[1], MPU6050.accY);
		MPU6050.accY = (int16_t)ekf[1].out;
		Com_Kalman_1(&ekf[2], MPU6050.accZ);
		MPU6050.accZ = (int16_t)ekf[2].out;
		//3.�Խ��ٶȽ���һ�׵�ͨ�˲�
		static int16_t lastGyro[3] = {0};
		MPU6050.gyroX =0.85 * lastGyro[0] + 0.15 * MPU6050.gyroX;
		lastGyro[0] = MPU6050.gyroX;
		MPU6050.gyroY =0.85 * lastGyro[1] + 0.15 * MPU6050.gyroY;
		lastGyro[1] = MPU6050.gyroY;
		MPU6050.gyroZ =0.85 * lastGyro[2] + 0.15 * MPU6050.gyroZ;
		lastGyro[2] = MPU6050.gyroZ;
		//4.�Ƕȼ��㣺��Ԫ������
		
		
}

void Get_MPU_Offset(void)
{
		u8 gyro_i = 30;
		u32 buff[6] = {0};
		//�жϴ������Ƿ�ֹ��ȡǰ�����β�ֵ
		const int8_t MAX_GYRO_QUIET =  5;
		const int8_t MIN_GYRO_QUIET = -5;
		int16_t lastGyro[3] = {0}; 	//������һ����������������
		int16_t	Err_Gyro[3] = {0};	//����ÿ�μ�������ƫ��ֵ
		//�����ж�30��
		while(gyro_i--)
		{
				delay_ms(10);
				//������������
				do
				{
						//��ȡMPUֵ
						MPU_Get_data();
						//����ƫ��
						Err_Gyro[0] = MPU6050.gyroX - lastGyro[0];
						Err_Gyro[1] = MPU6050.gyroY - lastGyro[1];
						Err_Gyro[2] = MPU6050.gyroZ - lastGyro[2];
						//���浱ǰֵ
						lastGyro[0] = MPU6050.gyroX;
						lastGyro[1] = MPU6050.gyroY;
						lastGyro[2] = MPU6050.gyroZ;
					
				}while(	Err_Gyro[0] < MIN_GYRO_QUIET || Err_Gyro[0] > MAX_GYRO_QUIET
						||	Err_Gyro[1] < MIN_GYRO_QUIET || Err_Gyro[1] > MAX_GYRO_QUIET
						||	Err_Gyro[2] < MIN_GYRO_QUIET || Err_Gyro[2] > MAX_GYRO_QUIET
								);
				
		}
	
		//ȡ���ԭʼֵ��ȡƽ��ֵ
		for(u16 i = 0; i < 356; i++)
		{
				delay_ms(10);
				//��ȡMPUֵ
				MPU_Get_data();
				if(i >= 100)
				{

						buff[0] += MPU6050.accX;
						buff[1] += MPU6050.accY;
						buff[2] += MPU6050.accZ - 16384;
						buff[3] += MPU6050.gyroX;
						buff[4] += MPU6050.gyroY;
						buff[5] += MPU6050.gyroZ;
				}
				
		}
		//��256�Σ�����8λ
		for(u8 i = 0; i < 6; i++)
		{
				mpu_Offset[i] = buff[i] >> 8;
		}
		printf("err_gyro\r\n");
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
	u8 i; 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
	for(i=0;i<len;i++)
	{
		MPU_IIC_Send_Byte(buf[i]);	//��������
		if(MPU_IIC_Wait_Ack())		//�ȴ�ACK
		{
			MPU_IIC_Stop();	 
			return 1;		 
		}		
	}    
    MPU_IIC_Stop();	 
	return 0;	
} 
//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
 	MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((addr<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((addr<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	while(len)
	{
		if(len==1)*buf=MPU_IIC_Read_Byte(0);//������,����nACK 
		else *buf=MPU_IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++; 
	}    
    MPU_IIC_Stop();	//����һ��ֹͣ���� 
	return 0;	
}
//IICдһ���ֽ� 
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 reg,u8 data) 				 
{ 
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	if(MPU_IIC_Wait_Ack())	//�ȴ�Ӧ��
	{
		MPU_IIC_Stop();		 
		return 1;		
	}
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	MPU_IIC_Send_Byte(data);//��������
	if(MPU_IIC_Wait_Ack())	//�ȴ�ACK
	{
		MPU_IIC_Stop();	 
		return 1;		 
	}		 
    MPU_IIC_Stop();	 
	return 0;
}
//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 reg)
{
	u8 res;
    MPU_IIC_Start(); 
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|0);//����������ַ+д����	
	MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
    MPU_IIC_Send_Byte(reg);	//д�Ĵ�����ַ
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ��
    MPU_IIC_Start();
	MPU_IIC_Send_Byte((MPU_ADDR<<1)|1);//����������ַ+������	
    MPU_IIC_Wait_Ack();		//�ȴ�Ӧ�� 
	res=MPU_IIC_Read_Byte(0);//��ȡ����,����nACK 
    MPU_IIC_Stop();			//����һ��ֹͣ���� 
	return res;		
}

