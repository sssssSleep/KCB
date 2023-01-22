#ifndef __OPTICALFLOW_H
#define __OPTICALFLOW_H


#include "usart.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

typedef struct

{

int16_t vx;

int16_t vy;

uint8_t qual;
	
}OPT_Data;

static void _Optflow_Prase(void);
void Opt_init(void);
void UpdateOptData(void);
OPT_Data Get_Opt_Data(void);

#endif  
