#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

#include "UartDriver.h"
#include "mpu6050.h"
#include "opticalflow.h"
#include "SPL06_001.h"
#include <stdio.h>
#include "ultrasonic.h"

#define TASK_1_TIME 10
#define TASK_2_TIME 10
#define TASK_3_TIME 10
#define TASK_4_TIME 10

#define DELAY_NUM   						20				//任务延时最大个数
#define FIRST_TASK  						TASK_1 		//第一个任务
#define LAST_TASK	 						TASK_6   	//最后一个任务
#define PACKET_UPDATE_INTERVAL 	200   		//数据包更新间隔（ms）
#define PACKET_SEND_INTERVAL		500				//数据包发送间隔（ms）
//任务延迟宏，一个任务仅能拥有一个，放在最开始，作为间隔运行，等待时间会切换到别的任务
#define TaskDelay_ms(x) if(DelayMs(x,taskid)) return;	
//状态枚举
enum state
{
	TASK_1,
	TASK_2,
	TASK_3,
	TASK_4,
	TASK_5,
	TASK_6,
};
enum callstate
{
	NO_RESPNOD,
	RESPNODED,
	ACCOMPLISHED,
};
typedef struct
{
	uint8_t taskid;
	uint32_t ms;
	uint32_t lstime;
}_TaskDelay;
typedef struct
{
	enum state state;
	enum state lstate;
	enum state cstate;
	enum callstate cdstate;
	uint8_t called_flag;
}_StatemachineData;
//状态机循环体
void loop(void);
//状态机相关函数
uint8_t CallTask(enum state state);
void DelayListInit(void);
//任务 任务ID是为了记录任务延时宏
void task1(uint16_t taskid);
void task2(uint16_t taskid);
void task3(uint16_t taskid);
void task4(uint16_t taskid);
void task5(uint16_t taskid);
void task6(uint16_t taskid);
//任务延时函数
uint8_t DelayMs(uint32_t ms , uint16_t taskid);
#endif  
