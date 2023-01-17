#ifndef __UARTDRIVE_H
#define __UARTDRIVE_H

#include "usart.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
//定义数据结构体
typedef struct
{
	uint8_t roll;
	uint8_t pitch;
	uint8_t throttle;
	uint8_t yaw;
	uint8_t mode;
}UFD;
void fly(void);
uint8_t SendOnePackage();
uint8_t UpdatePackage();

//数据结构体读写操作
void WriteThrottle_SD(uint8_t parm);
uint8_t ReadThrottle_SD();

void WriteYaw_SD(uint8_t parm);
uint8_t ReadYaw_SD();

void WritePitch_SD(uint8_t parm);
uint8_t ReadPitch_SD();

void WriteRoll_SD(uint8_t parm);
uint8_t ReadRoll_SD();

void WriteMode_SD(uint8_t parm);
uint8_t ReadMode_SD();
#endif  
