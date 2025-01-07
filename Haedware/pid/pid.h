#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef volatile struct
{
		float desired;			//����ֵ
		float prevError;		//�ϴ�ƫ��
		float integ;				//���ֵ�ۼ�
		float kp;						//P����
		float ki;						//i����
		float kd;						//d����
		float measured;			//ʵ�ʲ���ֵ
		float out;					//pid���
}PidProject;

void ResetPID(PidProject ** PidProjects, u8 len);
void PID_Updata(PidProject *pid, float dt);
void CasecadePID(PidProject *pidAngle, PidProject *pidRate, float	 dt);

#endif
