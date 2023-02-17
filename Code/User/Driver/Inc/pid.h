#ifndef __PID_H
#define __PID_H


typedef struct
{
    float target_val;           //目标值
    float actual_val;        		//实际值
    float err;             			//定义偏差值
    float err_last;          		//定义上两个偏差值
		float err_next;          		//定义上一个偏差值
    float Kp,Ki,Kd;          		//定义比例、积分、微分系数
    float integral;          		//定义积分值
		float integral_min;					//积分限幅
		float integral_max;					//积分限幅
		float output_val_max;				//输出限幅
		float output_val_min;				//输出限幅
}pid_t;
void PID_Init(pid_t* pid,float Kp,float Ki,float Kd,float integral_max,float integral_min,float output_val_max,float output_val_min);
float PID_pos_realize(pid_t* pid,float temp_val,float sensor_val);
float PID_inc_realize(pid_t* pid,float temp_val,float sensor_val);

#endif  
