#ifndef __OPTICALFLOW_H
#define __OPTICALFLOW_H


#include "usart.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

#define OPT_Kp            5.0f
#define OPT_Ki            0.0f
#define OPT_Kd            0.0f
#define OPT_INTEGRAL_MAX   200.0f
#define OPT_INTEGRAL_MIN  -200.0f
#define OPT_OUTPUT_MAX     130.0f
#define OPT_OUTPUT_MIN    -130.0f

#define Axis_Num           3
#define Num								 10
#define M_PI_F 3.1416f
enum 
{
	_PITCH = 0,
	_ROLL,
	_YAW,
};
typedef struct

{

int16_t vx;

int16_t vy;

int16_t x;

int16_t y;
	
uint32_t last_time;
uint32_t inv_time;
	
uint8_t qual;
	
}OPT_Data;
typedef struct
{
	int16_t pixel_flow_x_integral;
	int16_t pixel_flow_y_integral;
	uint8_t qual;
}_opt_origin_data;

typedef struct
{
	int16_t x;
	int16_t y;
	uint8_t qual;
}_opt_filter_data;
typedef struct
{
	float Input_Butter[3];
	float Output_Butter[3];
	
}Butter_BufferData;
typedef struct
{
	float a[3];
	float b[3];
	
}Butter_Parameter;
typedef struct
{
 float Position[Axis_Num];//位置估计量
 float Speed[Axis_Num];//速度估计量
 float Acceleration[Axis_Num];//加速度估计量
 float Pos_History[Axis_Num][Num];//历史惯导位置
 float Vel_History[Axis_Num][Num];//历史惯导速度
 float Acce_Bias[Axis_Num];//惯导加速度漂移量估计量
 }SINS;

 typedef struct
{
  float Last_Time;
  float Now_Time;
  float Time_Delta;
  uint16_t Time_Delta_INT;//单位ms
}Testime;
 typedef struct
{
  float x;
	float y;
}Vector2f;
float OpticalFlow_Rotate_Complementary_Filter(float optflow_gyro,float gyro,uint8_t axis);
static uint8_t Optflow_Prase();
void Opt_init(void);
void UpdateOptData(void);
OPT_Data Get_Opt_Data(void);
uint8_t OPT_Y_PID(float target_val);
uint8_t OPT_X_PID(float target_val);
void  OpticalFlow_CF(float flow_height,Vector2f accel,Vector2f flow);
void Optflow_Task(void);
#endif  
