/*
描述本文件

作者+日期
*/

#include "statemachine.h"
#define CHANNEL 12
static uint32_t ltime = 0;	//状态机时钟
static uint32_t testtime = 0;	//测试时钟
static uint32_t testtime2 = 0;	//测试时钟
static uint8_t fly_flag = 0;//起飞标志位
static _TaskDelay delay_list[DELAY_NUM];//任务延时表
static float fdata[CHANNEL];
MPU_data mpudata;
MPU_data mpudata_kalman;
extern OPT_Data opt_data;
static _StatemachineData FSMD;

/***************************************************
*@brief:  状态机主循环																			 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void loop()
{
	int res = 1;
	//Init
	DelayListInit();
  MPU_Init();
  //printf("\r\nmpu dmp = %d\r\n", mpu_dmp_init());
	mpu_dmp_init();
	Opt_init();
	spl0601_init();
	Altimeter_init();
	//HAL_Delay(2000);
	//testtime = HAL_GetTick();
	//testtime2 = HAL_GetTick();
	//start
	while(1)
	{
		UpdateTime();
		switch(FSMD.state)
		{
			case TASK_1:
						UpdateTime();
						task1(0x11);
						break;
			case TASK_2:
						UpdateTime();
						task2(0x22);
						break;
			case TASK_3:
						UpdateTime();
						task3(0x33);
						break;
			case TASK_4:
						UpdateTime();
						task4(0x44);
						break;	
			case TASK_5:
						UpdateTime();
						task5(0x55);
						break;
			case TASK_6:
						UpdateTime(); 
						//task6(0x66);
						break;				
		
		}
		UpdateState();
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
	TaskDelay_ms(100);
	UpdatePackage();
	SendOnePackage();
	//send_ultrasonic_com();
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}
/***************************************************
*@brief:  任务三																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task3(uint16_t taskid)
{
	TaskDelay_ms(200);
	//WriteThrottle_SD(SPL06_001_PID(100));//上下
	WriteThrottle_SD(0x81);
	//WritePitch_SD(OPT_X_PID(0));//前后
	WritePitch_SD(0x80);//前后
//	WriteRoll_SD(OPT_Y_PID(0));//左右
	WriteRoll_SD(0x80);//左右
	UpdatePackage();
	SendOnePackage();
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}
/***************************************************
*@brief:  任务四	更新传感器信息																			 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task4(uint16_t taskid)
{
	TaskDelay_ms(20);
	Update_MPU_Data();
	mpudata = Get_MPU_Data();
	mpudata_kalman = Get_MPU_Data_Kalman();
	Optflow_Task();
	user_spl0601_get();
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}

 /***************************************************
*@brief:  任务五	打印调试信息																			 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task5(uint16_t taskid)
{

	TaskDelay_ms(100);	
	//printf("task %x start %d ms\r\n",taskid,HAL_GetTick());
	opt_data = Get_Opt_Data();
	extern SPL_Data spl_data;
	fdata[0] = mpudata.pitch;
	fdata[1] = mpudata.roll;
	fdata[2] = mpudata.yaw;
	fdata[3] = mpudata_kalman.pitch;
	fdata[4] = mpudata_kalman.roll;
	fdata[5] = mpudata_kalman.yaw;
	fdata[6] = mpudata.yaw + mpudata.linear_cor_val_k*(HAL_GetTick() - mpudata.base_time);
	//printf("time : %d \r\n",HAL_GetTick() - mpudata.base_time);
	fdata[7] = spl_data.baro_height;
	fdata[8] = spl_data.baro_height_b;
	fdata[9] = opt_data.vy;
	fdata[10] = opt_data.vx;
	fdata[11] = opt_data.qual;
	print_data();
	//send_ultrasonic_com();
	//printf("%x%x%x",(uint32_t)mpudata.pitch,(uint32_t)mpudata.roll,(uint32_t)mpudata.yaw);
	//printf("hight = %f",get_hight());
	//printf("opt_PID_X = %x  opt_PID_Y = %x\r\n",OPT_X_PID(0),OPT_Y_PID(0));
	//printf("SPL_PID = %x\r\n",SPL06_001_PID(50));
	//printf("pitch = %f  roll = %f  yaw = %f\r\n",mpudata.pitch,mpudata.roll,mpudata.yaw);
	//printf("ax    = %d  ay   = %d  az  = %d\r\n",mpudata.ax,mpudata.ay,mpudata.az);
	//printf("temp  = %d \r\n",mpudata.temp);
	//printf("vy = : %d  vx = : %d  qual = : %d\r\n",opt_data.vy,opt_data.vx,opt_data.qual);
	//printf("task %x end %d ms\r\n",taskid,HAL_GetTick());
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}
 /***************************************************
*@brief:  任务六																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task6(uint16_t taskid)
{
	TaskDelay_ms(30);
	Altimeter_Prase();
	printf("%f\r\n",get_hight());
	//printf("vy = : %d  vx = : %d  qual = : %d\r\n",optdata.vy,optdata.vx,optdata.qual);
	//user_spl0601_get();
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}
