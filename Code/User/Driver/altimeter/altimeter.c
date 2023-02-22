
#include "altimeter.h"



float hight;
#if TF200F
static uint8_t data[14];
#endif
#if GYUS42V1
static uint8_t data[14];
#endif

void Altimeter_init()
{
	#if TF200F
	HAL_UART_Receive_DMA(&huart6,data,14);
	#endif
	#if GYUS42V1
	HAL_UART_Receive_DMA(&huart6,data,14);
	#endif
}

void Altimeter_Prase()
{
	 int s_i = 0;
		uint16_t x;
#if TF200F
		for(int i = 0;i<8;i++)
		{
			if(data[i] == 0xfe)
				s_i = i;
		}
				if(data[s_i+0]==0x01&&data[s_i+1]==0x03&&data[s_i+2]==0x02)
				{
				 x = (data[s_i+3]<<8) | (data[s_i+4]<<0);							
         hight = (float)x;
				}
#endif
#if GYUS42V1
	uint8_t sum;
		for(int i = 0;i<8;i++)
		{
			if(data[i] == 0x5a && data[i+1] == 0x5a)
				s_i = i;
		}
				if(data[s_i+0]==0x5a&&data[s_i+1]==0x5a&&data[s_i+2]==0x45)
				{
				sum = (data[s_i+0]+data[s_i+1]+data[s_i+2]+data[s_i+3]+data[s_i+4]+data[s_i+5]);
					if(sum == data[s_i+6])
					{
						x = (data[s_i+4]<<8) | (data[s_i+5]<<0);							
						hight = (float)x;
					}
				}
#endif
}



float get_hight()
{
	return hight;
}
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart == &huart6)
//	{
//		hight = ((ult_buff[0]<<16) + (ult_buff[1]<<8) + (ult_buff[2]<<0))/1000.0f;
//		printf("rx : %x %x %x\r\n",ult_buff[0],ult_buff[1],ult_buff[2]);
//		send_ultrasonic_com();
//	}
//}

