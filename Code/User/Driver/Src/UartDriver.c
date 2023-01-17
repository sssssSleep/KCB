/*
本文件为通过uart与无人机通讯，并作控制
LHQ 2022.01.16 
*/
#include "UartDriver.h"
uint8_t date[10];
static uint8_t loopstate_send = 1;


void test()
{
			HAL_Delay(8000);
			date[8] = 1;
			GetPackage(date,0x80,0x80,0x80,0x80);
      SendOnePackage(date); //发送命令2，定高模式
      HAL_Delay(100);
      GetPackage(date,0x80,0x80,0x80,0xFF);
      SendOnePackage(date); //发送命令4，电机开始慢转
      HAL_Delay(1000);
			GetPackage(date,0x80,0x80,0x80,0x80);
      SendOnePackage(date); //发送命令2，定高模式
      HAL_Delay(100);
       GetPackage(date,0x80,0x80,0x80,0xFF);
      SendOnePackage(date); //发送命令4，起飞，上升
      HAL_Delay(1500);  //持续上升1.5秒
      
      //uav_cmd[2][1]=0x78; //正常悬停时此值为0x80,无人机会往右侧飞，所以微调参数
      //uav_cmd[2][6]=uav_cmd[2][1];
      //for(uint8_t i=2;i<6;i++)  //更改数据后需要重新计算命令2的校验码，存在uav_cmd[2][6]中
      //  uav_cmd[2][6]=uav_cmd[2][6]^uav_cmd[2][i];
      
     /* for(uint8_t i=0;i<100;i++)  //悬停10秒
      { send_char_array(&huart2,uav_cmd[2],8); //保持当前高度 需要不断地发命令 
        HAL_Delay(100);  
      }
      
      uav_cmd[3][3]=64;
      uav_cmd[3][6]=uav_cmd[3][1];
      for(uint8_t i=2;i<6;i++)  //更改数据后需要重新计算命令3的校验码，存在uav_cmd[3][6]中
        uav_cmd[3][6]=uav_cmd[3][6]^uav_cmd[3][i];
      
      for(uint8_t i=0;i<50;i++)
      { send_char_array(&huart2,uav_cmd[3],8); //半速下降    //持续5秒
        HAL_Delay(100);  
      }
  
      send_char_array(&huart2,uav_cmd[11],8); //电机停转
*/
}

void fly()
{
	HAL_Delay(5000);
	//SendPackageLoop(date,0,0);
	for(int i = 0;i < 4; i++)
	{
		date[8] = 1;
		GetPackage(date,0x80,0x80,0x80,0x80);
		SendOnePackage(date);
		HAL_Delay(1000);
		GetPackage(date,0x80,0x80,0x80,0xFF);
		SendOnePackage(date);
		HAL_Delay(1000);
		
	}
	while(1)
	{
		GetPackage(date,0x79,0x79,0x80,0x83);
		SendOnePackage(date);
		HAL_Delay(500);
	}
	
	date[8] = 1;
		GetPackage(date,0x80,0x80,0x80,0xFF);
		for(int i = 0;i < 20; i++)
		{
			SendOnePackage(date);
			date[8] = 0;
			HAL_Delay(100);
		}
	
	date[8] = 1;
	GetPackage(date,0x80,0x80,0x80,0x80);
	for(int i = 0;i < 10; i++)
	{
		SendOnePackage(date);
		date[8] = 0;
		HAL_Delay(100);
	}
	date[8] = 1;
		GetPackage(date,0x80,0x80,0x80,0xFF);
		for(int i = 0;i < 20; i++)
		{
			SendOnePackage(date);
			date[8] = 0;
			HAL_Delay(100);
		}
	
		date[8] = 1;
		GetPackage(date,0x80,0x80,0x80,0xFF);
		for(int i = 0;i < 2; i++)
		{
			SendOnePackage(date);
			date[8] = 0;
			HAL_Delay(100);
		}
		
	date[8] = 1;
	GetPackage(date,0x80,0x80,0x80,0xFF);
	for(int i = 0;i < 1500; i++)
	{
		SendOnePackage(date);
		date[8] = 0;
		HAL_Delay(100);
	}
}
//注释
/***************************************************
*@brief:  发送一个数据包																				 
*@param:  数据包的数组指针                                     
*@retval: 0	：发送失败,数据包已被发送
					1	：发送成功
					2	：发送失败，数据包格式错误
					
*@author: lhq 2023.1.16                                        
****************************************************/
uint8_t SendOnePackage(uint8_t* pDate)
{
	if(pDate[8] == 0)
	{
		if(pDate[0] == 0x66)//发送包尾
		{
			for(int i = 0;i < 8;i++)
			{
				HAL_UART_Transmit_DMA(&huart2,pDate,8);
				//HAL_UART_Transmit(&huart1,&pDate[i],1,0xff);
			}
			pDate[8] = 1;
			return 1;
		}
		else
			return 2;
	}
	else
		return 0;
}

/***************************************************
*@brief:  循环发送数据包																				 
*@param:  数据包的数组指针                                     
*@retval: 1	：完成发送
*@author: lhq 2023.1.16                                        
****************************************************/
uint8_t SendPackageLoop(uint8_t* pDate,uint32_t frequency,uint32_t times)
{
	HAL_UART_Transmit_DMA(&huart2,pDate,8);
	return 1;
}
/***************************************************
*@brief:  对数据进行打包																				 
*@param:  buff		：数组指针，用来存放这个包
					roll		：横滚角
					pitch		：俯仰角
					yaw			：偏航角
					throttle：油门
*@retval: 0 : 打包失败，该数据未被发送       
					1	：打包成功
*@author: lhq 2023.1.16                                         
****************************************************/
uint8_t GetPackage(uint8_t* buff,uint8_t roll,uint8_t pitch,uint8_t yaw,uint8_t throttle)
{
	if(buff[8] != 0)
	{
		buff[0] = 0x66;		
		buff[1] = roll;		
		buff[2] = pitch;	
		buff[3] = throttle;
		buff[4] = yaw;
		buff[5] = 0x00;
		buff[6] = buff[1] ^ buff[2] ^ buff[3] ^ buff[4] ^ buff[5];
		buff[7] =	0x99;
		buff[8] = 0;//是否被发送，检查位
		return 1;
	}
	else
		return 0;
}
