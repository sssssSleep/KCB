/*
描述本文件

作者+日期
*/

#include "opticalflow.h"
#include "main.h"
#include "math.h"
#include "Print.h"
#include "pid.h"
#include "MPU6050.h"
#include "SPL06_001.h"
static uint8_t _flow_data_buff[18];
OPT_Data opt_data;
static pid_t opt_pid_x;
static pid_t opt_pid_y;
extern MPU_data _mpu_data;
extern SPL_Data spl_data;
_opt_origin_data opt_origin_data;
Vector2f gyro_filter_data;
_opt_filter_data opt_filter_data;
Vector2f opt_gyro_data;
Vector2f opt_gyro_filter_data;
Butter_Parameter OpticalFlow_Parameter,OpticalFlow_Gyro_Parameter;
Butter_BufferData Buffer_OpticalFlow[2],Buffer_OpticalFlow_Gyro[2];
extern float hight;
//注释格式
/***************************************************
*@brief:  描述																				 
*@param:  参数                                      
*@retval: 返回值                                        
*@author: 作者+日期                                         
****************************************************/
float constrain_float(float amt, float low, float high) 
{
//    if (isnan(amt))                  //51里面没有这个库函数，需要自己实现
//    {
//        return (low+high)*0.5f;
//    }
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
	float fr = sample_frequent / cutoff_frequent;
	float ohm = tanf(M_PI_F / fr);
	float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
	if (cutoff_frequent <= 0.0f) {
		// no filtering
		return;
	}
	LPF->b[0] = ohm * ohm / c;
	LPF->b[1] = 2.0f * LPF->b[0];
	LPF->b[2] = LPF->b[0];
        LPF->a[0]=1.0f;
	LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
	LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

void Test_Period(Testime *Time_Lab)
{
   Time_Lab->Last_Time=Time_Lab->Now_Time;
   Time_Lab->Now_Time=HAL_GetTick();//单位ms
   Time_Lab->Time_Delta=Time_Lab->Now_Time-Time_Lab->Last_Time;
   Time_Lab->Time_Delta_INT=(uint16_t)(Time_Lab->Time_Delta);
}

float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{   
    /* 加速度计Butterworth滤波 */
	/* 获取最新x(n) */
        static int LPB_Cnt=0;
        Buffer->Input_Butter[2]=curr_input;
        if(LPB_Cnt>=100)
        {
	/* Butterworth滤波 */
        Buffer->Output_Butter[2]=
         Parameter->b[0] * Buffer->Input_Butter[2] 
        +Parameter->b[1] * Buffer->Input_Butter[1]
	+Parameter->b[2] * Buffer->Input_Butter[0] 
        -Parameter->a[1] * Buffer->Output_Butter[1] 
        -Parameter->a[2] * Buffer->Output_Butter[0];
        }
        else
        {
          Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
          LPB_Cnt++;
        }
	/* x(n) 序列保存 */
        Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
        Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
	/* y(n) 序列保存 */
        Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
        Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
        
        
        return Buffer->Output_Butter[2];
}

void Opt_init()
{
	HAL_UART_Receive_DMA(&huart2,_flow_data_buff,18);
	PID_Init(&opt_pid_x,OPT_Kp,OPT_Ki,OPT_Kd,OPT_INTEGRAL_MAX,OPT_INTEGRAL_MIN,OPT_OUTPUT_MAX,OPT_OUTPUT_MIN);
	PID_Init(&opt_pid_y,OPT_Kp,OPT_Ki,OPT_Kd,OPT_INTEGRAL_MAX,OPT_INTEGRAL_MIN,OPT_OUTPUT_MAX,OPT_OUTPUT_MIN);
	Set_Cutoff_Frequency(50, 20,&OpticalFlow_Parameter);
  Set_Cutoff_Frequency(50, 8,&OpticalFlow_Gyro_Parameter);//同步相位
}

static uint8_t Optflow_Prase()//50hz
{
    uint8_t sum;
	 int s_i = 0;
		for(int i = 0;i<10;i++)
		{
			if(_flow_data_buff[i] == 0xfe)
				s_i = i;
		}
				if(_flow_data_buff[s_i+0]==0xfe&&_flow_data_buff[s_i+1]==0x04&&_flow_data_buff[s_i+8]==0xAA)
				{
					sum =_flow_data_buff[s_i+2]+_flow_data_buff[s_i+3]+_flow_data_buff[s_i+4]+_flow_data_buff[s_i+5]; 
					if(sum ==_flow_data_buff[s_i+6])
					{													
          opt_origin_data.pixel_flow_y_integral=(int16_t)(_flow_data_buff[s_i+3]<<8)|_flow_data_buff[s_i+2];
          opt_origin_data.pixel_flow_x_integral=(int16_t)(_flow_data_buff[s_i+5]<<8)|_flow_data_buff[s_i+4];
          opt_origin_data.pixel_flow_x_integral*=(-1);
					//opt_data.vx = opt_origin_data.pixel_flow_x_integral;
					//opt_data.vy = opt_origin_data.pixel_flow_y_integral;
          //opt_origin_data.integration_timespan= (int16_t)(OpticalFlow_Ringbuf.Ring_Buff[i+7]<<8)|OpticalFlow_Ringbuf.Ring_Buff[i+6];
          opt_origin_data.qual=_flow_data_buff[s_i+7]; 
					}
        opt_filter_data.x=LPButterworth(opt_origin_data.pixel_flow_x_integral,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
        opt_filter_data.y=LPButterworth(opt_origin_data.pixel_flow_y_integral,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);   
        //opt_data.x=(opt_origin_data.pixel_flow_x_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm
        //opt_data.y=(opt_origin_data.pixel_flow_y_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm     
        //opt_data.dt=(int16_t)(opt_origin_data.integration_timespan*0.001f);//单位ms
        opt_data.qual=opt_origin_data.qual;    
        opt_gyro_data.x=opt_filter_data.x*0.075f;//光流角速度rad/s
        opt_gyro_data.y=opt_filter_data.y*0.075f;//光流角速度rad/s 				
        gyro_filter_data.x=LPButterworth(_mpu_data.vpitch,&Buffer_OpticalFlow_Gyro[0],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度
        gyro_filter_data.y=LPButterworth(_mpu_data.vroll,&Buffer_OpticalFlow_Gyro[1],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度
        opt_gyro_filter_data.x=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.x,gyro_filter_data.x,'x');//光流角速度与陀螺仪角速度融合 
        opt_gyro_filter_data.y=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.y,gyro_filter_data.y,'y'); //光流角速度与陀螺仪角速度融合 
				//opt_data.vx = opt_gyro_filter_data.x * hight; // cm/s
				//opt_data.vy = opt_gyro_filter_data.y * hight; // cm/s
        return 1;
      }
  return 0;
}

uint8_t Optflow_Is_Okay=0;
void Optflow_Task(void)
{

	Vector2f SINS_Accel_Body;
  Optflow_Is_Okay=Optflow_Prase();
	SINS_Accel_Body.x = -_mpu_data.ax;
	SINS_Accel_Body.y = _mpu_data.ay;
  //OpticalFlow_CF(spl_data.baro_height*10,SINS_Accel_Body,opt_gyro_filter_data);
	OpticalFlow_CF(hight,SINS_Accel_Body,opt_gyro_filter_data);
	
}


float OpticalFlow_Rotate_Complementary_Filter(float optflow_gyro,float gyro,uint8_t axis)
{
  float optflow_gyro_filter=0;  
    if(axis=='x') optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-3.0f,3.0f);//4
    else optflow_gyro_filter=optflow_gyro-constrain_float(gyro,-3.0f,3.0f);
 
  return optflow_gyro_filter;
}





SINS OpticalFlow_SINS;
Testime Optical_Delta;
Vector2f OpticalFlow_Position={0};
Vector2f OpticalFlow_Speed={0};
Vector2f OpticalFlow_Speed_Err={0};
Vector2f OpticalFlow_Position_Err={0};
uint16_t Optflow_Sync_Cnt=5;
float CF_Parameter[2]={0.05f,0.0f};//光流互补滤波权重 0.1  0.1   0.08  0
//光流位置融合权重给为0，表示不加入修正位置修正，因为低成本光流模块漂移较大，亦可以给较小值如0.2f
#define Optical_Output_Dt  0.02f//50hz

  void  OpticalFlow_CF(float flow_height,Vector2f accel,Vector2f flow)
{
  float use_height=0;
  float optical_dt=0;
  Vector2f speed_delta={0};
  Test_Period(&Optical_Delta);
  optical_dt=Optical_Delta.Time_Delta/1000.0f;
  use_height = flow_height; 
  if(Optflow_Is_Okay==1)//存在数据光流更新时，20ms一次
  {  
    OpticalFlow_Speed.x=flow.x*use_height;//光流速度
    OpticalFlow_Speed.y=flow.y*use_height;//光流速度
    OpticalFlow_Position.x+=OpticalFlow_Speed.x*Optical_Output_Dt;//光流位移
    OpticalFlow_Position.y+=OpticalFlow_Speed.y*Optical_Output_Dt;//光流位移
    Optflow_Is_Okay=0;
    OpticalFlow_Position_Err.x=OpticalFlow_Position.x-OpticalFlow_SINS.Pos_History[_PITCH][Optflow_Sync_Cnt];
    OpticalFlow_Position_Err.y=OpticalFlow_Position.y-OpticalFlow_SINS.Pos_History[_ROLL][Optflow_Sync_Cnt];
		OpticalFlow_Speed_Err.x=OpticalFlow_Speed.x-OpticalFlow_SINS.Vel_History[_PITCH][Optflow_Sync_Cnt];
    OpticalFlow_Speed_Err.y=OpticalFlow_Speed.y-OpticalFlow_SINS.Vel_History[_ROLL][Optflow_Sync_Cnt];
		
		OpticalFlow_Speed_Err.x=constrain_float(OpticalFlow_Speed_Err.x,-200,200);
		OpticalFlow_Speed_Err.y=constrain_float(OpticalFlow_Speed_Err.y,-200,200);
  }
  else
  {
    OpticalFlow_Speed_Err.x=0;
    OpticalFlow_Speed_Err.y=0;
    OpticalFlow_Position_Err.x=0;
    OpticalFlow_Position_Err.y=0;
  }
  
  OpticalFlow_SINS.Acceleration[_PITCH]=-accel.x;//惯导加速度沿载体机头
  OpticalFlow_SINS.Acceleration[_ROLL]=accel.y;//惯导加速度沿载体横滚（机头右侧）
  
	speed_delta.x=OpticalFlow_SINS.Acceleration[_PITCH]*optical_dt;
  speed_delta.y=OpticalFlow_SINS.Acceleration[_ROLL]*optical_dt;    
  
	OpticalFlow_SINS.Position[_PITCH]+=OpticalFlow_SINS.Speed[_PITCH]*optical_dt
    +0.5f*speed_delta.x*optical_dt+CF_Parameter[1]*OpticalFlow_Position_Err.x;
  OpticalFlow_SINS.Position[_ROLL]+=OpticalFlow_SINS.Speed[_ROLL]*optical_dt
    +0.5f*speed_delta.y*optical_dt+CF_Parameter[1]*OpticalFlow_Position_Err.y;

  OpticalFlow_SINS.Speed[_PITCH]+=OpticalFlow_SINS.Acceleration[_PITCH]*optical_dt+CF_Parameter[0]*OpticalFlow_Speed_Err.x;
  OpticalFlow_SINS.Speed[_ROLL]+=OpticalFlow_SINS.Acceleration[_ROLL]*optical_dt+CF_Parameter[0]*OpticalFlow_Speed_Err.y; 
   
	
	for(uint16_t i=Num-1;i>0;i--)
	{
		OpticalFlow_SINS.Pos_History[_ROLL][i]=OpticalFlow_SINS.Pos_History[_ROLL][i-1];
		OpticalFlow_SINS.Pos_History[_PITCH][i]=OpticalFlow_SINS.Pos_History[_PITCH][i-1];
		OpticalFlow_SINS.Vel_History[_ROLL][i]=OpticalFlow_SINS.Vel_History[_ROLL][i-1];
		OpticalFlow_SINS.Vel_History[_PITCH][i]=OpticalFlow_SINS.Vel_History[_PITCH][i-1]; 		
	}   
	OpticalFlow_SINS.Pos_History[_ROLL][0]=OpticalFlow_SINS.Position[_ROLL];
  OpticalFlow_SINS.Pos_History[_PITCH][0]=OpticalFlow_SINS.Position[_PITCH]; 
  OpticalFlow_SINS.Vel_History[_ROLL][0]=OpticalFlow_SINS.Speed[_ROLL];
  OpticalFlow_SINS.Vel_History[_PITCH][0]=OpticalFlow_SINS.Speed[_PITCH];  	 
}







//static void _Optflow_Prase()//50hz
//{
//		uint8_t sum;
//	 int s_i = 0;
//		for(int i = 0;i<10;i++)
//		{
//			if(_flow_data_buff[i] == 0xfe)
//				s_i = i;
//		}
//				if(_flow_data_buff[s_i+0]==0xfe&&_flow_data_buff[s_i+1]==0x04&&_flow_data_buff[s_i+8]==0xAA)
//				{
//					sum =_flow_data_buff[s_i+2]+_flow_data_buff[s_i+3]+_flow_data_buff[s_i+4]+_flow_data_buff[s_i+5]; 
//					if(sum ==_flow_data_buff[s_i+6])
//					{							
//						_opt_origin_data.vy=(int16_t)(_flow_data_buff[s_i+3]<<8)|_flow_data_buff[s_i+2];
//						_opt_origin_data.vx=(int16_t)(_flow_data_buff[s_i+5]<<8)|_flow_data_buff[s_i+4];
//						_opt_origin_data.vx = -1*_opt_origin_data.vx;
//						//opt_origin_data.pixel_flow_y_integral*=(-1);
//						//opt_origin_data.integration_timespan= (int16_t)(_flow_data_buff[7]<<8)|_flow_data_buff[6];
//						_opt_origin_data.qual=_flow_data_buff[s_i+7]; 
//					}
//					//opt_filter_data.x=LPButterworth(opt_origin_data.pixel_flow_x_integral,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
//					//opt_filter_data.y=LPButterworth(opt_origin_data.pixel_flow_y_integral,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);   
//					//opt_data.x=(opt_origin_data.pixel_flow_x_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm
//					//opt_data.y=(opt_origin_data.pixel_flow_y_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm     
//					//opt_data.dt=(int16_t)(opt_origin_data.integration_timespan*0.001f);//单位ms
//				 // opt_data.qual=opt_origin_data.qual;    
//					//opt_gyro_data.x=opt_filter_data.x*0.075f;//光流角速度rad/s
//					//opt_gyro_data.y=opt_filter_data.y*0.075f;//光流角速度rad/s 				
//					//gyro_filter_data.x=LPButterworth(Roll_Gyro,&Buffer_OpticalFlow_Gyro[0],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度
//					//gyro_filter_data.y=LPButterworth(Pitch_Gyro,&Buffer_OpticalFlow_Gyro[1],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度
//					//opt_gyro_filter_data.x=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.x,gyro_filter_data.x,'x');//光流角速度与陀螺仪角速度融合 
//					//opt_gyro_filter_data.y=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.y,gyro_filter_data.y,'y'); //光流角速度与陀螺仪角速度融合 
//	}
//}
void UpdateOptData(void)
{
	Optflow_Prase();
}
OPT_Data Get_Opt_Data(void)
{
	return opt_data;
}

uint8_t OPT_X_PID(float target_val)
{
	float res = 0.0f;
	
	res = PID_inc_realize(&opt_pid_x,target_val,OpticalFlow_SINS.Speed[_PITCH]);
	//printf("opt_x: res = %f ures = %x vx = %f\r\n",res,res>=0 ? (uint8_t)((int)res) : (uint8_t)((int)(-res)),OpticalFlow_SINS.Speed[_PITCH]);
	if(res>=0)
	{
		if((int)res > 0x7F)
			return 0xFF;
		return 0x80 + (uint8_t)((int)res);
	}
	else if(res<0)
	{
		res = -res;
		if((int)res > 0x7F)
			return 0x00;
		return 0x80 - (uint8_t)((int)res);
	}
	return 0x80;
}
uint8_t OPT_Y_PID(float target_val)
{
	float res = 0.0f;
	
	res = PID_inc_realize(&opt_pid_y,target_val,(float)OpticalFlow_SINS.Speed[_ROLL]);
	//printf("opt_y: res = %f  ures  = %x  vy = %f\r\n",res,res>=0 ? (uint8_t)((int)res) : (uint8_t)((int)(-res)),OpticalFlow_SINS.Speed[_ROLL]);
	if(res>=0)
	{
		if((int)res > 0x7F)
			return 0xFF;
		return 0x80 + (uint8_t)((int)res);
	}
	else if(res<0)
	{
		res = -res;
		if((int)res > 0x7F)
			return 0x00;
		return 0x80 - (uint8_t)((int)res);
	}
	return 0x80;
}

