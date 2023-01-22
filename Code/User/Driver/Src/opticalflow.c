/*
描述本文件

作者+日期
*/

#include "opticalflow.h"
#include "main.h"
#include "Print.h"
static uint8_t _flow_data_buff[18];
static OPT_Data _opt_origin_data;
//注释格式
/***************************************************
*@brief:  描述																				 
*@param:  参数                                      
*@retval: 返回值                                        
*@author: 作者+日期                                         
****************************************************/

void Opt_init()
{
	HAL_UART_Receive_DMA(&huart2,_flow_data_buff,18);
}
static void _Optflow_Prase()//50hz
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
						_opt_origin_data.vy=(int16_t)(_flow_data_buff[s_i+3]<<8)|_flow_data_buff[s_i+2];
						_opt_origin_data.vx=(int16_t)(_flow_data_buff[s_i+5]<<8)|_flow_data_buff[s_i+4];
						//opt_origin_data.pixel_flow_y_integral*=(-1);
						//opt_origin_data.integration_timespan= (int16_t)(_flow_data_buff[7]<<8)|_flow_data_buff[6];
						_opt_origin_data.qual=_flow_data_buff[s_i+7]; 
					}
					//opt_filter_data.x=LPButterworth(opt_origin_data.pixel_flow_x_integral,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
					//opt_filter_data.y=LPButterworth(opt_origin_data.pixel_flow_y_integral,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);   
					//opt_data.x=(opt_origin_data.pixel_flow_x_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm
					//opt_data.y=(opt_origin_data.pixel_flow_y_integral*opticalflow_high)/10000.0f;//单位:乘以高度单位mm后为实际位移mm     
					//opt_data.dt=(int16_t)(opt_origin_data.integration_timespan*0.001f);//单位ms
				 // opt_data.qual=opt_origin_data.qual;    
					//opt_gyro_data.x=opt_filter_data.x*0.075f;//光流角速度rad/s
					//opt_gyro_data.y=opt_filter_data.y*0.075f;//光流角速度rad/s 				
					//gyro_filter_data.x=LPButterworth(Roll_Gyro,&Buffer_OpticalFlow_Gyro[0],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度
					//gyro_filter_data.y=LPButterworth(Pitch_Gyro,&Buffer_OpticalFlow_Gyro[1],&OpticalFlow_Gyro_Parameter)/57.3f;//陀螺仪相位同步角速度
					//opt_gyro_filter_data.x=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.x,gyro_filter_data.x,'x');//光流角速度与陀螺仪角速度融合 
					//opt_gyro_filter_data.y=OpticalFlow_Rotate_Complementary_Filter(opt_gyro_data.y,gyro_filter_data.y,'y'); //光流角速度与陀螺仪角速度融合 
	}
}
void UpdateOptData(void)
{
	_Optflow_Prase();
}
OPT_Data Get_Opt_Data(void)
{
	return _opt_origin_data;
}