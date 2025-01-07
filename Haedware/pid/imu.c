#include "imu.h"
#include "math.h"


const float RtA = 57.2957795f;
const float Gyro_G = 0.03051756f*2;
const float Gyro_Gr = 0.0005326f*2;

extern _stMPU MPU6050;
extern _stAngle Angle;
_stIMU IMU;

static float NormAccz;
typedef volatile struct
{
		float q0;
		float q1;
		float q2;
		float q3;
} Quaternion;


float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
	
	x2 = number * 0.5F;
	y = number;
	i =*(long*) &y;
	i=0x5f3759df -( i>>1);
	y=*(float*)&i;
	y =y*( threehalfs - (x2*y*y));	//1st iteration(��һ��ţ�ٵ���)
	return y;
}

void GetAngle(const _stMPU *pMPU, _stAngle *pAngle, float dt )
{
		volatile struct V
		{
				float x;
				float y;
				float z;

		}Gravity, Acc, Gyro, AccGravity;

		static struct V GyroIntegError = {0};
		static float KpDef = 0.7f;
		static float KiDef = 0.0003f;

		static Quaternion NumQ = {1, 0, 0, 0};
		float q0_t, q1_t, q2_t, q3_t;
		float NormQuat;
		float HalfTime = dt * 0.5f;

		//��ȡ��Ч��ת�����е���������
		Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
		Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
		Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);
		//���ٶȹ�һ��
		NormQuat = Q_rsqrt(squa(MPU6050.accX) + squa(MPU6050.accY) + squa(MPU6050.accZ));
		Acc.x = pMPU->accX * NormQuat;
		Acc.y = pMPU->accY * NormQuat;
		Acc.z = pMPU->accZ * NormQuat;
		//������˵ó���ֵ
		AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
		AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
		AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);
		//�������ٶȻ��ֲ������ٶȵĲ���ֵ
		GyroIntegError.x += AccGravity.x * KiDef;
		GyroIntegError.y += AccGravity.y * KiDef;
		GyroIntegError.z += AccGravity.z * KiDef;

		//���ٶ��ںϼ��ٶȻ��ֲ���ֵ
		Gyro.x = pMPU->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
		Gyro.y = pMPU->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
		Gyro.z = pMPU->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;
		// -�������������������Ԫ��
		q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
		q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
		q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
		q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;
		NumQ.q0 += q0_t;
		NumQ.q1 += q1_t;
		NumQ.q2 += q2_t;
		NumQ.q3 += q3_t;
		
		//��Ԫ����һ��
		NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
		NumQ.q0 *= NormQuat;
		NumQ.q1 *= NormQuat;
		NumQ.q2 *= NormQuat;
		NumQ.q3 *= NormQuat;
		{
				float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;
				float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;
				float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2; 
#ifdef YAW_GYRO
				*(float *)pAngle = atan2f(2*NumQ.q1 * NumQ.q2 + 2* NumQ.q0 * NumQ.q3, 1 - 2*NumQ.q2 - 2* NumQ.q3 * NumQ.q3)*RtA;
#else
				float yaw_G = pMPU->gyroZ * Gyro_G;
				if ((yaw_G > 1.0f) || (yaw_G < -1.0f)) //����̫С������Ϊ�Ǹ��ţ�����ƫ������
				{
						pAngle->yaw += yaw_G * dt; //���ٶȻ��ֳ�ƫ����
				}

#endif
				pAngle->pitch = asin(vecxZ) * RtA; //������
				pAngle->roll = atan2f(vecyZ, veczZ) * RtA; //�����
				NormAccz = pMPU->accX * vecxZ + pMPU->accY * vecyZ + pMPU->accZ * veczZ; 
		}
}

float GetAccz(void)
{
		return NormAccz;
		
}


//float Q_rsqrt(float number)
//{
//    long i;
//    float x2, y;
//    const float threehalfs = 1.5F;

//    x2 = number * 0.5F;
//    y = number;
//    i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float*)&i;
//    y = y * (threehalfs - (x2 * y * y));  // 1st iteration of Newton's method
//    return y;
//}

//void GetAngle(const _stMPU *pMPU, _stAngle *pAngle, float dt)
//{
//    volatile struct V
//    {
//        float x;
//        float y;
//        float z;
//    } Gravity, Acc, Gyro, AccGravity;

//    static struct V GyroIntegError = {0};
//    static float KpDef = 0.8f;
//    static float KiDef = 0.0003f;

//    static Quaternion NumQ = {1, 0, 0, 0};
//    float q0_t, q1_t, q2_t, q3_t;
//    float NormQuat;
//    float HalfTime = dt * 0.5f;

//    // Extract gravity vector from quaternion
//    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
//    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
//    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

//    // Normalize accelerometer data
//    NormQuat = Q_rsqrt(squa(MPU6050.accX) + squa(MPU6050.accY) + squa(MPU6050.accZ));
//    Acc.x = pMPU->accX * NormQuat;
//    Acc.y = pMPU->accY * NormQuat;
//    Acc.z = pMPU->accZ * NormQuat;

//    // Calculate cross product (acceleration - gravity)
//    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
//    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
//    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

//    // Integrate accelerometer data for gyroscope correction
//    GyroIntegError.x += AccGravity.x * KiDef;
//    GyroIntegError.y += AccGravity.y * KiDef;
//    GyroIntegError.z += AccGravity.z * KiDef;

//    // Combine accelerometer and gyroscope data
//    Gyro.x = pMPU->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
//    Gyro.y = pMPU->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
//    Gyro.z = pMPU->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

//    // Runge-Kutta method to update quaternion
//    q0_t = (-NumQ.q1 * Gyro.x - NumQ.q2 * Gyro.y - NumQ.q3 * Gyro.z) * HalfTime;
//    q1_t = (NumQ.q0 * Gyro.x - NumQ.q3 * Gyro.y + NumQ.q2 * Gyro.z) * HalfTime;
//    q2_t = (NumQ.q3 * Gyro.x + NumQ.q0 * Gyro.y - NumQ.q1 * Gyro.z) * HalfTime;
//    q3_t = (-NumQ.q2 * Gyro.x + NumQ.q1 * Gyro.y + NumQ.q0 * Gyro.z) * HalfTime;

//    NumQ.q0 += q0_t;
//    NumQ.q1 += q1_t;
//    NumQ.q2 += q2_t;
//    NumQ.q3 += q3_t;

//    // Normalize quaternion
//    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
//    NumQ.q0 *= NormQuat;
//    NumQ.q1 *= NormQuat;
//    NumQ.q2 *= NormQuat;
//    NumQ.q3 *= NormQuat;

//    // Calculate angles (pitch, roll, yaw)
//    float vecxZ = 2 * NumQ.q0 * NumQ.q2 - 2 * NumQ.q1 * NumQ.q3;
//    float vecyZ = 2 * NumQ.q2 * NumQ.q3 + 2 * NumQ.q0 * NumQ.q1;
//    float veczZ = 1 - 2 * NumQ.q1 * NumQ.q1 - 2 * NumQ.q2 * NumQ.q2;

//#ifdef YAW_GYRO
//    *(float *)pAngle = atan2f(2 * NumQ.q1 * NumQ.q2 + 2 * NumQ.q0 * NumQ.q3, 1 - 2 * NumQ.q2 - 2 * NumQ.q3 * NumQ.q3) * RtA;
//#else
//    float yaw_G = pMPU->gyroZ * Gyro_G;
//    if ((yaw_G > 1.0f) || (yaw_G < -1.0f))  // Ignore noise
//    {
//        pAngle->yaw += yaw_G * dt;  // Integrate gyro for yaw
//    }
//#endif

//    pAngle->pitch = asin(vecxZ) * RtA;  // Pitch angle
//    pAngle->roll = atan2f(vecyZ, veczZ) * RtA;  // Roll angle
//    NormAccz = pMPU->accX * vecxZ + pMPU->accY * vecyZ + pMPU->accZ * veczZ;
//}

//float GetAccz(void)
//{
//    return NormAccz;
//}

