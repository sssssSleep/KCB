/*
本文件为通过uart与无人机通讯，并作控制
LHQ 2022.01.16 
*/
#include "UartDriver.h"
#include "stdio.h"
static uint8_t data[10];//数据包存放数组
static UFD udata;				//数据结构体

/***************************************************
*@brief:  起飞函数																	 
*@param: 	无                                    
*@retval: 无			
*@author: lhq 2023.1.16                                        
****************************************************/
void fly()
{
			HAL_Delay(2000);
			udata.roll=0x80;
			udata.pitch=0x80;
			udata.throttle=0x80;
			udata.yaw=0x80;
			udata.mode=0x00;
			data[8] = 1;
			UpdatePackage();
	for(int i = 0;i < 3; i++)
	{
		WriteThrottle_SD(0x80);
		UpdatePackage();
		SendOnePackage();
		HAL_Delay(500);
		WriteThrottle_SD(0xFF);
		UpdatePackage();
		SendOnePackage();
		HAL_Delay(1000);
	}

}
//注释
/***************************************************
*@brief:  发送一个数据包																				 
*@param:  数据包的数组指针                                     
*@retval: 0	：发送失败,数据包已被发送
					1	：发送成功
					
*@author: lhq 2023.1.16                                        
****************************************************/
uint8_t SendOnePackage()
{
	if(data[8] == 0)
	{	
			while(HAL_UART_Transmit_DMA(&huart2,data,8) == HAL_BUSY)
			{
				huart2.gState = HAL_UART_STATE_READY;
				__HAL_UNLOCK(&huart2);//解锁
			}
				//HAL_UART_Transmit(&huart1,data,8,0xff);
			data[8] = 1;
			return 1;
	}
	else
		return 0;
}

/***************************************************
*@brief:  对数据进行打包																				 
*@param:  buff		：数组指针，用来存放这个包
					data		：数据包结构体
*@retval: 0 : 打包失败，该数据未被发送       
					1	：打包成功
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t UpdatePackage()
{
	if(data[8] != 0)
	{
		data[0] = 0x66;		
		data[1] = udata.roll;		
		data[2] = udata.pitch;	
		data[3] = udata.throttle;
		data[4] = udata.yaw;
		data[5] = udata.mode;
		data[6] = data[1] ^ data[2] ^ data[3] ^ data[4] ^ data[5];
		data[7] =	0x99;
		data[8] = 0;//是否被发送，检查位
		return 1;
	}
	else
		return 0;
}
/***************************************************
*@brief:  修改数据结构体的油门参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteThrottle_SD(uint8_t parm)
{	
	udata.throttle = parm;
}
/***************************************************
*@brief:  读取数据结构体的油门参数																				 
*@param:  无
*@retval: 油门参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadThrottle_SD()
{	
	return udata.throttle;
}
/***************************************************
*@brief:  修改数据结构体的横滚角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteRoll_SD(uint8_t parm)
{	
	udata.roll = parm;
}
/***************************************************
*@brief:  读取数据结构体的横滚角参数																				 
*@param:  无
*@retval: 横滚角
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadRoll_SD()
{	  
	return udata.roll;
}
/***************************************************
*@brief:  修改数据结构体的俯仰角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WritePitch_SD(uint8_t parm)
{	
	udata.pitch = parm;
}
/***************************************************
*@brief:  读取数据结构体的俯仰角参数																				 
*@param:  无
*@retval: 俯仰角参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadPitch_SD()
{	
	return udata.pitch;
}
/***************************************************
*@brief:  修改数据结构体的偏航角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteYaw_SD(uint8_t parm)
{	
	udata.yaw = parm;
}
/***************************************************
*@brief:  读取数据结构体的偏航角参数																				 
*@param:  无
*@retval: 偏航角参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadYaw_SD()
{	
	return udata.yaw;
}	

/***************************************************
*@brief:  修改数据结构体的偏航角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteMode_SD(uint8_t parm)
{	
	udata.mode = parm;
}
/***************************************************
*@brief:  读取数据结构体的偏航角参数																				 
*@param:  无
*@retval: 偏航角参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadMode_SD()
{	
	return udata.mode;
}	
