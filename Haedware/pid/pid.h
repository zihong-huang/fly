#ifndef __PID_H
#define __PID_H

#include "sys.h"

typedef volatile struct
{
		float desired;			//期望值
		float prevError;		//上次偏差
		float integ;				//误差值累加
		float kp;						//P参数
		float ki;						//i参数
		float kd;						//d参数
		float measured;			//实际测量值
		float out;					//pid输出
}PidProject;

void ResetPID(PidProject ** PidProjects, u8 len);
void PID_Updata(PidProject *pid, float dt);
void CasecadePID(PidProject *pidAngle, PidProject *pidRate, float	 dt);

#endif
