/*
描述本文件

作者+日期
*/

#include "pid.h"
#include "stdio.h"

//注释格式
/***************************************************
*@brief:  描述																				 
*@param:  参数                                      
*@retval: 返回值                                        
*@author: 作者+日期                                         
****************************************************/
void PID_Init(pid_t* pid,float Kp,float Ki,float Kd,float integral_max,float integral_min,float output_val_max,float output_val_min)
{
		pid->Kp = Kp;
		pid->Ki = Ki;
		pid->Kd = Kd;
		pid->actual_val = 0.0f;
		pid->err = 0.0f;
		pid->err_last = 0.0f;
		pid->err_next = 0.0f;
		pid->integral = 0.0f;
		pid->target_val = 0.0f;
		pid->integral_max = integral_max;
		pid->integral_min = integral_min;
		pid->output_val_max = output_val_max;
		pid->output_val_min = output_val_min;
	//printf("param : kp = %f  ki = %f  kd = %f\r\n",Kp,Ki,Kd);
	
	//printf("pid: kp = %f  ki = %f  kd = %f\r\n",pid->Kp,pid->Ki,pid->Kd);
	//printf("pid: integral_max = %f  integral_min = %f  output_val_max = %f\r  output_val_min = %f\r\n",pid->integral_max,pid->integral_min,pid->output_val_max,pid->output_val_min);
	
}
float PID_inc_realize(pid_t* pid,float temp_val,float sensor_val) 
{
	//printf("kp = %f  ki = %f  kd = %f\r\n",pid->Kp,pid->Ki,pid->Kd);
	/*传入目标值*/
	pid->target_val = temp_val;
	pid->actual_val = sensor_val;
	/*计算目标值与实际值的误差*/
  pid->err=pid->target_val-pid->actual_val;
	/*PID算法实现*/
	float increment_val = pid->Kp*(pid->err - pid->err_next) + pid->Ki*pid->err + pid->Kd*(pid->err - 2 * pid->err_next + pid->err_last);
		//printf("err = %f  res = %f\r\n",pid->err,increment_val);
	/*累加*/
	pid->actual_val += increment_val;
	/*传递误差*/
	pid->err_last = pid->err_next;
	pid->err_next = pid->err;
	/*返回当前实际值*/
	if(pid->actual_val>=pid->output_val_max)
		return pid->output_val_max;
	else if(pid->actual_val<=pid->output_val_min)
		return pid->output_val_min;
	else
		return pid->actual_val;
}

float PID_pos_realize(pid_t* pid,float temp_val,float sensor_val)
{
	//printf("kp = %f  ki = %f  kd = %f\r\n",pid->Kp,pid->Ki,pid->Kd);
	/*传入目标值*/
	pid->target_val = temp_val;
	pid->actual_val = sensor_val;
	/*计算目标值与实际值的误差*/
    pid->err=pid->target_val-pid->actual_val;
	/*误差累积*/
	if(pid->integral>pid->integral_max&&pid->err < 0.0f)
	{
		pid->integral+=pid->err;
	}
	else if(pid->integral<pid->integral_min&&pid->err > 0.0f)
	{
		pid->integral+=pid->err;
	}
	else
	{
		pid->integral+=pid->err;
	}
	/*PID算法实现*/
    pid->actual_val=pid->Kp*pid->err+pid->Ki*pid->integral+pid->Kd*(pid->err-pid->err_last);
	//printf("err = %f  res = %f  integral  =  %f\r\n",pid->err,pid->actual_val,pid->integral);
	/*误差传递*/
    pid->err_last=pid->err;
	/*返回当前实际值*/
    if(pid->actual_val>=pid->output_val_max)
		return pid->output_val_max;
	else if(pid->actual_val<=pid->output_val_min)
		return pid->output_val_min;
	else
		return pid->actual_val;
}

