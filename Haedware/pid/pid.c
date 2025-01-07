#include "pid.h"

//pid公式 = p*公差 + I * 偏差的积分 + p * 偏差的变化率
//					p*（实际值-期望值） + I * （累加->偏差值 * dt） + D * (（偏差-上次偏差）/dt)

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
		float temp_err;		//本次偏差
		float dri;				
		/*1、计算偏差值*/
		temp_err = pid->desired - pid->measured;
		/*2、计算积分、偏差累积和*/
		pid->integ += temp_err * dt;
		/*3、计算微分、偏差的变化率*/
		dri = (temp_err - pid->prevError) / dt;
		/*4、结果保存到out里，本次偏差值保存到“上次偏差”字段*/
		pid->out = pid->kp * temp_err + pid->ki * pid->integ + pid->kp * dri;
		pid->prevError = temp_err;
	
}


void CasecadePID(PidProject *pidAngle, PidProject *pidRate, float	 dt)
{
		/*1、角度外环进行PID处理*/
		PID_Updata(pidAngle,dt);
		/*2、外环的输出，赋值给内环的期望值*/
		pidRate->desired = pidAngle->out;
		/*3、对内环进行PID计算*/
		PID_Updata(pidRate,dt);
}	


