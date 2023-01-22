/*
描述本文件

作者+日期
*/

#include "statemachine.h"

static enum state state = TASK_1;//创建枚举变量
static uint32_t ltime = 0;	//状态机时钟
static uint8_t fly_flag = 0;//起飞标志位
static uint32_t delay_list[DELAY_NUM][3] = { {0} };//任务延时表（可改为结构体数组）
static MPU_data mpudata;
static OPT_Data optdata;
//注释格式
/***************************************************
*@brief:  更新时间																				 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void UpdateTime()
{
	ltime = HAL_GetTick();
}
/***************************************************
*@brief:  按顺序切换状态																				 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void UpdateState()
{
	if(state == LAST_TASK)
	{
		state = FIRST_TASK;
	}
	else
		state++;
}
//uint32_t GetNowTime()
//{
//	return HAL_GetTick();
//}

//uint32_t GetLastTime()
//{
//	return ltime;
//}
/***************************************************
*@brief:  获取现在时刻与ltime时刻的时间间隔																				 
*@param:  无                                      
*@retval: 时间间隔                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
uint32_t GetIntervalTime()
{
	return (HAL_GetTick() - ltime);
}
/***************************************************
*@brief:  状态机主循环																			 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void loop()
{
	//Init
	MPU_Init();
  printf("\r\n%d\r\n", mpu_dmp_init());
	Opt_init();
	//HAL_Delay(5000);
	//start
	while(1)
	{
		UpdateTime();
		switch(state)
		{
			case TASK_1:
						UpdateTime();
						task1(0x11);
						UpdateState();
						break;
			case TASK_2:
						UpdateTime();
						task2(0x22);
						UpdateState();
						break;
			case TASK_3:
						UpdateTime();
						task3(0x33);
						UpdateState();
						break;
			case TASK_4:
						UpdateTime();
						task4(0x44);
						UpdateState();
						break;	
			case TASK_5:
						UpdateTime();
						task5(0x55);
						UpdateState();
						break;
			case TASK_6:
						UpdateTime();
						task6(0x66);
						UpdateState();
						break;				
		
		}
	}
}
/***************************************************
*@brief:  任务一																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task1(uint16_t taskid)
{
	
	if(!fly_flag)
	{
			fly();
			fly_flag = 1;
	}
}
/***************************************************
*@brief:  任务二																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task2(uint16_t taskid)
{
	TaskDelay_ms(500);
	UpdatePackage();
	SendOnePackage();
}
/***************************************************
*@brief:  任务三																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task3(uint16_t taskid)
{
	TaskDelay_ms(500);
	WriteThrottle_SD(0x80);
	WritePitch_SD(0x82);
	WriteRoll_SD(0x78);
}
/***************************************************
*@brief:  任务四	更新mpu信息																			 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task4(uint16_t taskid)
{
	TaskDelay_ms(20);
	Update_MPU_Data();
	mpudata = Get_MPU_Data();
}

 /***************************************************
*@brief:  任务五	打印调试信息																			 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task5(uint16_t taskid)
{
	TaskDelay_ms(900);
	printf("pitch = %f  roll = %f  yaw = %f\r\n",mpudata.pitch,mpudata.roll,mpudata.yaw);
	printf("ax    = %d  ay   = %d  az  = %d\r\n",mpudata.ax,mpudata.ay,mpudata.az);
	printf("temp  = %d \r\n",mpudata.temp);
	printf("vy = : %d  vx = : %d  qual = : %d\r\n",optdata.vy,optdata.vx,optdata.qual);
}
 /***************************************************
*@brief:  任务六																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task6(uint16_t taskid)
{
	TaskDelay_ms(20);
	UpdateOptData();
	optdata = Get_Opt_Data();
}
/***************************************************
*@brief:  任务延时函数																				 
*@param:  ms		:  延时时间
					taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
uint8_t DelayMs(uint32_t ms , uint16_t taskid)
{
	uint8_t listnum = 0;
	uint8_t exist_flag = 0;
	for(int i = 0;i < DELAY_NUM;i++)//查找该任务是否存在延时
	{
			if(delay_list[i][0] == taskid && delay_list[i][1] == ms)
			{
				listnum = i;
				exist_flag = 1;
				break;
			}
	}
	if(!exist_flag)//查找该任务是否存在延时
	{
		for(int i = 0;i < DELAY_NUM;i++)
		{
			if(delay_list[i][0] == 0)
			{
				delay_list[i][0] = taskid;
				delay_list[i][1] = ms;
				delay_list[i][2] = HAL_GetTick();//获取延时开始时刻
				listnum = i;
				break;
			}
		}
	}
	if(HAL_GetTick()-delay_list[listnum][2] >= delay_list[listnum][1])//判断时间间隔是否达到延时时间
	{
		//delay_list[listnum][0] = 0;//释放任务延时
		delay_list[listnum][2] = HAL_GetTick();//循环
		return 0 ;//结束延时中
	}
	else
	{
		return 1;//还在延时中
	}
}
