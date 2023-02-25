/*
描述本文件

作者+日期
*/

#include "statemachine.h"
#define CHANNEL 10
static uint32_t ltime = 0;	//状态机时钟
static uint32_t testtime = 0;	//测试时钟
static uint32_t testtime2 = 0;	//测试时钟
static uint8_t fly_flag = 0;//起飞标志位
static _TaskDelay delay_list[DELAY_NUM];//任务延时表
static float fdata[CHANNEL];
Target_Val target = {
.hover = 1,
};
MPU_data mpudata;

extern OPT_Data opt_data;
static _StatemachineData FSMD;
float height;
//注释格式
/***************************************************
*@brief:  更新时间																				 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void DelayListInit()
{
	for(int i = 0;i < DELAY_NUM;i++)
	{	
		delay_list[i].lstime = 0;
		delay_list[i].ms = 0;
		delay_list[i].taskid = 0;
	}
}
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
*@brief:  打印FireWater格式数据																				 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void print_data()
{
	for(int i = 0;i<CHANNEL;i++)
	{
		printf("%f",fdata[i]);
		if(i+1 != CHANNEL)
		{
			printf(",");
		}
	}
	printf("\r\n");
}

/***************************************************
*@brief:  按顺序切换状态																				 
*@param:  无                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void UpdateState()
{
	if(FSMD.called_flag)//判断是需要调用任务
	{
		switch(FSMD.cdstate)//任务调用状态
		{
			case NO_RESPNOD:
				FSMD.cdstate = RESPNODED;//若未响应则更新为已响应
				break;
			case RESPNODED:
				FSMD.cdstate = ACCOMPLISHED;//若已响应则更新为已完成
				break;
			case ACCOMPLISHED:
				break;
		}
		if(FSMD.cdstate == ACCOMPLISHED)//若已完成则恢复顺序状态更新
		{
				if(FSMD.lstate == LAST_TASK)
				{
					FSMD.state = FIRST_TASK;
				}
				else
				{
					FSMD.state = FSMD.lstate++;
				}
				FSMD.called_flag = 0;//完成一次任务调用
			
		}
		return ;
	}
	//无任务调用时顺序更新
	if(FSMD.state == LAST_TASK)
	{
		FSMD.lstate = FSMD.state;
		FSMD.state = FIRST_TASK;
	}
	else
	{
		FSMD.lstate = FSMD.state;
		FSMD.state++;
	}
}

/***************************************************
*@brief:  调用任务																				 
*@param:  要调用的任务                                     
*@retval: 0 : 切换成功
					1 ：已有切换任务
*@author: 梁辉强 2023.1.17                                        
****************************************************/
uint8_t CallTask(enum state state)
{
	if(FSMD.called_flag)//已有切换任务
	{
		if(FSMD.cstate == FSMD.state && FSMD.cdstate == RESPNODED)//如果是在被调用任务中再次调用任务，被调用任务已被响应
		{
			//printf("in task %d called task %d,ctask is task %d,ltask = task %d\r\n",FSMD.state+1,state+1,FSMD.cstate+1,FSMD.lstate+1);
			FSMD.cstate = state;//则更新被调用任务
			FSMD.cdstate = NO_RESPNOD;//更新调用响应状态
			FSMD.state = state;//更新下一个要进入的任务 为 被调用任务
			return 0;
		}
		return 1;
	}
	else//如果没有
	{
		//printf("in task %d called task %d\r\n",FSMD.state+1,state+1);
		FSMD.cdstate = NO_RESPNOD;//更新调用响应状态
		FSMD.lstate = FSMD.state;//记录跳出位置
		FSMD.cstate = state;//记录被调用任务
		FSMD.state = state;//更新下一个要进入的任务 为 被调用任务
		FSMD.called_flag = 1;	//更新标志位
		return 0;
	}
}

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
	int res = 1;
	//Init
	
	DelayListInit();
  MPU_Init();
	mpu_dmp_init();
	Opt_init();
	spl0601_init();
	Altimeter_init();

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
						task6(0x66);
						break;				
		
		}
		UpdateState();
	}
}
/***************************************************
*@brief:  起飞																				 
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
*@brief:  主任务																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task2(uint16_t taskid)
{
	TaskDelay_ms(50);
	task_main(target);
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}
/***************************************************
*@brief:  发送控制指令																			 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task3(uint16_t taskid)
{
	TaskDelay_ms(100);
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
	//mpudata_kalman = Get_MPU_Data_Kalman();
	Optflow_Task();
	//user_spl0601_get();
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
	fdata[0] = mpudata.pitch;
	fdata[1] = mpudata.roll;
	fdata[2] = mpudata.yaw;
	fdata[3] = opt_data.x;
	fdata[4] = opt_data.y;
	fdata[5] = 
	fdata[6] = opt_data.vy;
	fdata[7] = opt_data.vx;
	fdata[8] = opt_data.qual;
	fdata[9] = get_user_height();
	print_data();

}
 /***************************************************
*@brief:  更新高度计																				 
*@param:  taskid： 任务id                                      
*@retval: 无                                     
*@author: 梁辉强 2023.1.17                                        
****************************************************/
void task6(uint16_t taskid)
{
	TaskDelay_ms(100);
	Altimeter_Prase();
	height = get_user_height();
	//printf("vy = : %d  vx = : %d  qual = : %d\r\n",optdata.vy,optdata.vx,optdata.qual);
	//user_spl0601_get();
	//printf("task %x used %d ms\r\n",taskid,GetIntervalTime());
}

void task_main(Target_Val tval)
{
	if(tval.hover == 1)//悬停任务 计算1m Pid  光流定点
	{
		WriteThrottle_SD(0x81);
		WritePitch_SD(0x81);//前后
		WriteRoll_SD(0x80);//左右
	}
	else //画圆任务 光流控制x轴速度，以低速运动，同时yaw轴以一定速度旋转 计算方式为 (vx/100)*57.3f  °/s 对vyaw进行pid计算
	{
		WriteThrottle_SD(0x81);
		WritePitch_SD(0x81);//前后
		WriteRoll_SD(0x80);//左右
		WriteYaw_SD(0x80);//旋转
	}
}
void PID_calculate(Target_Val val)
{
	
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
	if(FSMD.called_flag)//若存在调用任务
	{
		for(int i = 0;i < DELAY_NUM;i++)//查找任务id对应的延时
		{
			if(delay_list[i].taskid == taskid && delay_list[i].ms == ms)
			{
				listnum = i;
				break;
			}
		}
		delay_list[listnum].lstime = HAL_GetTick();//更新任务时间，避免连续两次调用
		return 0;//直接跳过一次延时
	}
	for(int i = 0;i < DELAY_NUM;i++)//查找该任务是否存在延时
	{
			if(delay_list[i].taskid == taskid && delay_list[i].ms == ms)
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
			if(delay_list[i].taskid == 0)
			{
				delay_list[i].taskid = taskid;
				delay_list[i].ms = ms;
				delay_list[i].lstime = HAL_GetTick();//获取延时开始时刻
				listnum = i;
				break;
			}
		}
	}
	if(HAL_GetTick()-delay_list[listnum].lstime >= delay_list[listnum].ms)//判断时间间隔是否达到延时时间
	{
		//delay_list[listnum][0] = 0;//释放任务延时
		//printf("task %x delay %d ms\r\n",taskid,HAL_GetTick()-delay_list[listnum].lstime);
		delay_list[listnum].lstime = HAL_GetTick();//循环
		return 0 ;//结束延时中
	}
	else
	{
		return 1;//还在延时中
	}
}
