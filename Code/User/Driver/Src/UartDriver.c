/*
本文件为通过uart与无人机通讯，并作控制
LHQ 2022.01.16 
*/
#include "UartDriver.h"
static uint8_t date[10];//数据包存放数组
static UFD udate;				//数据结构体

/***************************************************
*@brief:  起飞函数																	 
*@param: 	无                                    
*@retval: 无			
*@author: lhq 2023.1.16                                        
****************************************************/
void fly()
{
			HAL_Delay(5000);
			udate.roll=0x80;
			udate.pitch=0x80;
			udate.throttle=0x80;
			udate.yaw=0x80;
			udate.mode=0x00;
			date[8] = 1;
			UpdatePackage();
	for(int i = 0;i < 3; i++)
	{
		WriteThrottle_SD(0x80);
		UpdatePackage();
		SendOnePackage();
		HAL_Delay(1000);
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
	if(date[8] == 0)
	{	
				HAL_UART_Transmit_DMA(&huart2,date,8);
				//HAL_UART_Transmit(&huart1,date,8,0xff);
			
			date[8] = 1;
			return 1;
	}
	else
		return 0;
}

/***************************************************
*@brief:  对数据进行打包																				 
*@param:  buff		：数组指针，用来存放这个包
					date		：数据包结构体
*@retval: 0 : 打包失败，该数据未被发送       
					1	：打包成功
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t UpdatePackage()
{
	if(date[8] != 0)
	{
		date[0] = 0x66;		
		date[1] = udate.roll;		
		date[2] = udate.pitch;	
		date[3] = udate.throttle;
		date[4] = udate.yaw;
		date[5] = udate.mode;
		date[6] = date[1] ^ date[2] ^ date[3] ^ date[4] ^ date[5];
		date[7] =	0x99;
		date[8] = 0;//是否被发送，检查位
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
	udate.throttle = parm;
}
/***************************************************
*@brief:  读取数据结构体的油门参数																				 
*@param:  无
*@retval: 油门参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadThrottle_SD()
{	
	return udate.throttle;
}
/***************************************************
*@brief:  修改数据结构体的横滚角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteRoll_SD(uint8_t parm)
{	
	udate.roll = parm;
}
/***************************************************
*@brief:  读取数据结构体的横滚角参数																				 
*@param:  无
*@retval: 横滚角
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadRoll_SD()
{	  
	return udate.roll;
}
/***************************************************
*@brief:  修改数据结构体的俯仰角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WritePitch_SD(uint8_t parm)
{	
	udate.pitch = parm;
}
/***************************************************
*@brief:  读取数据结构体的俯仰角参数																				 
*@param:  无
*@retval: 俯仰角参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadPitch_SD()
{	
	return udate.pitch;
}
/***************************************************
*@brief:  修改数据结构体的偏航角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteYaw_SD(uint8_t parm)
{	
	udate.yaw = parm;
}
/***************************************************
*@brief:  读取数据结构体的偏航角参数																				 
*@param:  无
*@retval: 偏航角参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadYaw_SD()
{	
	return udate.yaw;
}	

/***************************************************
*@brief:  修改数据结构体的偏航角参数																				 
*@param:  parm:要改成的参数
*@retval: 无
*@author: lhq 2023.1.16                                         
****************************************************/
void WriteMode_SD(uint8_t parm)
{	
	udate.mode = parm;
}
/***************************************************
*@brief:  读取数据结构体的偏航角参数																				 
*@param:  无
*@retval: 偏航角参数
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t ReadMode_SD()
{	
	return udate.mode;
}	
