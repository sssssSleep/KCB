
#ifndef __ALTIMETER_H
#define __ALTIMETER_H
#include "main.h"
#include "stdio.h"
extern UART_HandleTypeDef huart6;


#define TF200F 0
#define GYUS42V1 1
void Altimeter_init();
void Altimeter_Prase();

float get_user_height();




#endif