#ifndef __STATEMACHINE_H
#define __STATEMACHINE_H

#include "UartDriver.h"

#define TASK_1_TIME 10
#define TASK_2_TIME 10
#define TASK_3_TIME 10
#define TASK_4_TIME 10

#define FIRST_TASK  TASK_1
#define LAST_TASK	 TASK_5
enum state
{
	TASK_1,
	TASK_2,
	TASK_3,
	TASK_4,
	TASK_5,
};
void loop(void);
void task1(void);
#endif  
