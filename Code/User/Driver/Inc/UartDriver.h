#ifndef __UARTDRIVE_H
#define __UARTDRIVE_H

#include "usart.h"
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;


void fly(void);
void test(void);
uint8_t SendPackageLoop(uint8_t* pDate,uint32_t frequency,uint32_t times);
uint8_t SendOnePackage(uint8_t* pDate);
uint8_t GetPackage(uint8_t* buff,uint8_t roll,uint8_t pitch,uint8_t yaw,uint8_t throttle);

#endif  
