#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

#include "common.h"
static int32_t killValue,AddValue;
typedef struct Task_s {
void (*f)(void *data); /* Task function */
void *data;  /* Private data pointer for this task */
}Task_t;

Task_t Array[10];

void sum(void *a);

void sum(void *a)
{
//int32_t myValue = (int32_t)a;
printf("this is in task 1\n");
return;
}
void mul(void *a)
{
printf("this is in task 2\n");
return;
}
void task3(void *a)
{
printf("this is in task 3\n");
return;
}
void task4(void *a)
{
printf("this is in task 4\n");
return;
}
void task5(void *a)
{
printf("this is in task 5\n");
return;
}
void task6(void *a)
{
printf("this is in task 6\n");
return;
}
void task7(void *a)
{
printf("this is in task 7\n");
return;
}
void task8(void *a)
{
printf("this is in task 8\n");
return;
}
void task9(void *a)
{
printf("this is in task 9\n");
return;
}
void task10(void *a)
{
printf("this is in task 10\n");
return;
}
/* Find the next task to run */
static int32_t TaskNext(void)
{
	int32_t i=0,currentTask=3;
	uint32_t count=0;
	i = currentTask;
	do 
		{
		i = (i + 1) % 10;
		count++;
		} while((Array[i].f == NULL)&& (count <= 10));
	return (count <= 10) ? i : -1;
}

void TaskShow()
{
	for(int i=0;i<10;i++)
	{
	if(Array[i].f != NULL)
	{
	(*Array[i].f)(Array[i].data);
	}
	else
	printf("No task available in this list\n");
	}
}
void TaskSwitcher()
{
	int32_t currentTask = TaskNext();
	Array[currentTask].f(Array[currentTask].data);
}
void TaskKill()
{
	for(int i=0;i<=10;i++)
	{
	if(i==(killValue-1))
	{
	Array[i].f    = NULL;
	Array[i].data = NULL;
	printf("task %ld has been killed\n",killValue);
	}
	}
printf("The new list is\n");
TaskShow();
}
void TaskKillAll()
{
	
	Array[0].f    = NULL;
	Array[1].f    = NULL;
	Array[2].f    = NULL;
	Array[3].f    = NULL;
	Array[4].f    = NULL;
	Array[5].f    = NULL;
	Array[6].f    = NULL;
	Array[7].f    = NULL;
	Array[8].f    = NULL;
	Array[9].f    = NULL;
printf("All the tasks are killed\n");
printf("The new list is\n");
TaskShow();
}
int32_t TaskAdd()
{
	if(AddValue==1)
		{Array[0].f    = sum;
		// Array[0].data = (void *)1;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==2)
		{Array[1].f    = mul;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==3)
		{Array[2].f    = task3;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==4)
		{Array[3].f    = task4;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==5)
		{Array[4].f    = task5;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==6)
		{Array[5].f    = task6;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==7)
		{Array[6].f    = task7;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==8)
		{Array[7].f    = task8;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==9)
		{Array[8].f    = task9;
		printf("task %ld has been added successfully\n",AddValue);}
	else if(AddValue==10)
		{Array[9].f    = task10;
		printf("task %ld has been added successfully\n",AddValue);}
	else
		{printf("No slots available\n");}

return -1;
}
int32_t TaskAddAll()
{
		Array[0].f    = sum;
		Array[1].f    = mul;
		Array[2].f    = task3;
		Array[3].f    = task4;
		Array[4].f    = task5;
		Array[5].f    = task6;
		Array[6].f    = task7;
		Array[7].f    = task8;
		Array[8].f    = task9;
		Array[9].f    = task10;
		printf("All the tasks have been added successfully\n");
return -1;
}
ParserReturnVal_t CmdTaskShow(int mode)
{

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

	TaskShow();
  return CmdReturnOk;
}

ADD_CMD("taskshow",CmdTaskShow,"            Task_Executive")
ParserReturnVal_t CmdTaskadd(int mode)
{

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
	fetch_int32_arg(&AddValue);
	TaskAdd();
  return CmdReturnOk;
}

ADD_CMD("taskadd",CmdTaskadd,"            Task_Executive")
ParserReturnVal_t CmdTaskKill(int mode)
{

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;
	fetch_int32_arg(&killValue);
	TaskKill();
  return CmdReturnOk;
}

ADD_CMD("taskkill",CmdTaskKill,"            Task_Executive")
ParserReturnVal_t CmdTaskSwitcher(int mode)
{

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

	TaskSwitcher();
  return CmdReturnOk;
}

ADD_CMD("taskswitch",CmdTaskSwitcher,"            Task_Executive")
ParserReturnVal_t CmdTaskaddall(int mode)
{

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

	TaskAddAll();
  return CmdReturnOk;
}

ADD_CMD("taskaddall",CmdTaskaddall,"            Task_Executive")
ParserReturnVal_t CmdTaskkillall(int mode)
{

  if(mode != CMD_INTERACTIVE) return CmdReturnOk;

	TaskKillAll();
  return CmdReturnOk;
}

ADD_CMD("taskkillall",CmdTaskkillall,"            Task_Executive")
