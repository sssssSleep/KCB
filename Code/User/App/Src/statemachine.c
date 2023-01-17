/*
描述本文件

作者+日期
*/

#include "statemachine.h"

static enum state state = TASK_1;
static uint32_t ltime = 0;
//注释格式
/***************************************************
*@brief:  描述																				 
*@param:  参数                                      
*@retval: 返回值                                        
*@author: 作者+日期                                         
****************************************************/
void UpdateTime()
{
	ltime = HAL_GetTick();
}
void UpdateState()
{
	if(state == LAST_TASK)
	{
		state = FIRST_TASK;
	}
	else
		state++;
}
uint32_t GetNowTime()
{
	return HAL_GetTick();
}

uint32_t GetLastTime()
{
	return ltime;
}
uint32_t GetIntervalTime()
{
	return (HAL_GetTick() - ltime);
}
void loop()
{
	UpdateTime();
	switch(state)
	{
		case TASK_1:
					UpdateTime();
					task1();
					UpdateState();
					break;
		case TASK_2:
					UpdateTime();
					UpdateState();
					break;
		case TASK_3:
					UpdateTime();
					UpdateState();
					break;
		case TASK_4:
					UpdateTime();
					UpdateState();
					break;	
		case TASK_5:
					UpdateTime();
					UpdateState();
					break;	
	
	}
}
void task1()
{
	fly();
	if(GetIntervalTime() >= TASK_1_TIME)
		return;
	
}
