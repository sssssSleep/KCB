/*
描述本文件

作者+日期
*/

#include "ultrasonic.h"
uint8_t ult_buff[3];
//float hight;
static uint8_t com = 0xA0; 
//注释格式
/***************************************************
*@brief:  描述																				 
*@param:  参数                                      
*@retval: 返回值                                        
*@author: 作者+日期                                         
****************************************************/

//void send_ultrasonic_com()
//{
//	printf("s = %d\r\n",HAL_UART_Transmit(&huart6,&com,1,0xff));
//	HAL_Delay(250);
//	printf("r = %d\r\n",HAL_UART_Receive(&huart6,ult_buff,3,0xff));
//	hight = ((ult_buff[0]<<16) + (ult_buff[1]<<8) + (ult_buff[2]<<0))/1000.0f;
//	printf("rx : %x %x %x\r\n",ult_buff[0],ult_buff[1],ult_buff[2]);
//	
//	//printf("send : %x %x %x",ult_buff[0],ult_buff[1],ult_buff[2]);
//}
//float get_hight()
//{
//	return hight;
//}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart6)
//	{
//		hight = ((ult_buff[0]<<16) + (ult_buff[1]<<8) + (ult_buff[2]<<0))/1000.0f;
//		printf("rx : %x %x %x\r\n",ult_buff[0],ult_buff[1],ult_buff[2]);
//		send_ultrasonic_com();
//	}
//}


	