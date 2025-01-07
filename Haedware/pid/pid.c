#include "pid.h"

//pid��ʽ = p*���� + I * ƫ��Ļ��� + p * ƫ��ı仯��
//					p*��ʵ��ֵ-����ֵ�� + I * ���ۼ�->ƫ��ֵ * dt�� + D * (��ƫ��-�ϴ�ƫ�/dt)

void ResetPID(PidProject ** PidProjects, u8 len)
{
		for(u8 i = 0; i < len; i++)
		{
				PidProjects[i]->desired = 0;
				PidProjects[i]->prevError = 0;
				PidProjects[i]->out = 0;
				PidProjects[i]->integ = 0;
		}
		
}

void PID_Updata(PidProject *pid, float dt)
{
		float temp_err;		//����ƫ��
		float dri;				
		/*1������ƫ��ֵ*/
		temp_err = pid->desired - pid->measured;
		/*2��������֡�ƫ���ۻ���*/
		pid->integ += temp_err * dt;
		/*3������΢�֡�ƫ��ı仯��*/
		dri = (temp_err - pid->prevError) / dt;
		/*4��������浽out�����ƫ��ֵ���浽���ϴ�ƫ��ֶ�*/
		pid->out = pid->kp * temp_err + pid->ki * pid->integ + pid->kp * dri;
		pid->prevError = temp_err;
	
}


void CasecadePID(PidProject *pidAngle, PidProject *pidRate, float	 dt)
{
		/*1���Ƕ��⻷����PID����*/
		PID_Updata(pidAngle,dt);
		/*2���⻷���������ֵ���ڻ�������ֵ*/
		pidRate->desired = pidAngle->out;
		/*3�����ڻ�����PID����*/
		PID_Updata(pidRate,dt);
}	


