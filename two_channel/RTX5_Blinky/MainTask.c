#include <stdio.h>
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "RTE_Components.h"             /* Component selection */
#include "Task.h"
#include "S32K144_API.h"
#include "SystemConfig.h"
#include "function.h"
#include "RTX_Config.h"
#include "s32_core_cm4.h"
#include  "Bms-Input-Output-Port.h"
#include "string.h"
#include CMSIS_device_header
#ifdef RTE_Compiler_EventRecorder
#include "EventRecorder.h"
#endif
/*=====================================================本文件使用的宏定义========================================================*/
/*===============================================================主任务参数定义==================================================*/
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];	      /*主任伍堆栈1024*/
const osThreadAttr_t app_main_attr = 
{
	.stack_mem  = &app_main_stk[0],		      /*主任务堆栈指针*/
	.stack_size = sizeof(app_main_stk),		      /*主任务堆栈尺寸*/
	.priority=MAINTASK_priority			      /*主任务优先级*/
};
/*===============================================================任务1参数定义==================================================*/
uint64_t app_Task1_stk[APP_Task1_STK_SZ / 8];	      /*任务1堆栈512字节*/
const osThreadAttr_t app_Task1_attr = 
{
	.stack_mem  = &app_Task1_stk[0],		      /*任务1务堆栈指针*/
	.stack_size = sizeof(app_Task1_stk),	      /*任务1务堆栈尺寸*/
	.priority=TASK1_priority			      /*任务1伍优先级*/
};
/*===============================================================任务2参数定义==================================================*/
uint64_t app_Task2_stk[APP_Task2_STK_SZ / 8];	      /*任务2堆栈512字节*/
const osThreadAttr_t app_Task2_attr = 
{
	.stack_mem  = &app_Task2_stk[0],		      /*任务2堆栈指针*/
	.stack_size = sizeof(app_Task2_stk),	      /*任务2堆栈尺寸*/
	.priority=TASK2_priority			      /*任务2优先级*/
};
/*===============================================================任务3参数定义==================================================*/
uint64_t app_Task3_stk[APP_Task3_STK_SZ / 8];	      /*任务3堆栈512字节*/
const osThreadAttr_t app_Task3_attr = 
{
	.stack_mem  = &app_Task3_stk[0],		      /*任务3堆栈指针*/
	.stack_size = sizeof(app_Task3_stk),	      /*任务3堆栈尺寸*/
	.priority=TASK3_priority			      /*任务3优先级*/
};
/*===============================================================任务4参数定义==================================================*/
uint64_t app_Task4_stk[APP_Task4_STK_SZ / 8];	      /*任务4堆栈512字节*/
const osThreadAttr_t app_Task4_attr = 
{
	.stack_mem  = &app_Task4_stk[0],		      /*任务4堆栈指针*/
	.stack_size = sizeof(app_Task4_stk),	      /*任务4堆栈尺寸*/
	.priority=TASK4_priority			      /*任务4优先级*/
};
/*====================================================全局变量定义==============================================================*/
osThreadId_t tid_Task1;                		      /* 任务1线程句柄 */
osThreadId_t tid_Task2;                		      /* 任务2线程句柄 */
osThreadId_t tid_Task3;                		      /* 任务3线程句柄 */
osThreadId_t tid_Task4;                		      /* 任务4线程句柄 */
/*====================================================全局函数声明==============================================================*/
/*===================================================任务声明===================================================================*/
extern 	void Task1(void *argument);		      /* 任务1*/
extern 	void Task2(void *argument);		      /* 任务2*/
extern 	void Task3(void *argument);		      /* 任务3*/
extern 	void Task4(void *argument);		      /* 任务4*/
//extern 	void Task5(void *argument);		      /* 任务5*/
//extern 	void Task6(void *argument);		      /* 任务6*/
/*-------------------------------------------------------------------------------------------------------------------------------
*                                                      主任务线程:
*说明:主任务用来完系统唤醒以后的主状态机控制，主要完成以下几个工作
*
*-------------------------------------------------------------------------------------------------------------------------------*/
__NO_RETURN void app_main (void *argument) 
{
	(void)argument;

#ifdef RTE_Compiler_EventRecorder
	EventRecorderInitialize(0U, 1U);
	EventRecorderEnable (EventRecordError, 0xF0U, 0xF8U);     /* RTOS Events */
	EventRecorderEnable (EventRecordAll, 0xF2U, 0xF2U);       /* Thread Events */
#endif
	/*=================================================生成所有系统相关任务===========================================*/
	tid_Task1 = osThreadNew (Task1, NULL, &app_Task1_attr);            /* 生成任务1 */
	if (tid_Task1 == NULL) { /* add error handling */ }
	tid_Task2 = osThreadNew (Task2, NULL, &app_Task2_attr);            /* 生成任务2 */
	if (tid_Task2 == NULL) { /* add error handling */ }
	tid_Task3 = osThreadNew (Task3, NULL, &app_Task3_attr);            /* 生成任务3 */
	if (tid_Task3 == NULL) { /* add error handling */ }
	tid_Task4 = osThreadNew (Task4, NULL, &app_Task4_attr);            /* 生成任务4 */
	if (tid_Task4 == NULL) { /* add error handling */ }
	/*=============================================相关任务本体开始执行===============================================*/
      /*================================================局部变量赋初值==================================================*/
      /*================================================================================================================*/
	for (;;) 
	{
            /*===========================系统时间累加============================================*/
            SysTickTime=osKernelGetTickCount() / ( OS_TICK_FREQ / 100) ;      /*系统检测节拍时间,精度10mS*/  
            SystemTimes = SysTickTime / 100;    /*获取系统上电后的运行时间,单位S*/
            /*===================================================================================*/
            #if   SYSTEM_DUTY_ENABLE==1                 /*启用LS3测量系统工作效率*/
                  LS3_DISABLE;            /*低端开关3禁止,PTD17=0*/  /*测量系统工作负载率*/
            #endif
            /*==================================================================================*/
		osDelay(TICK_20mS);	/*20mS时间片*/
            /*==================================================================================*/
            #if   SYSTEM_DUTY_ENABLE==1                 /*启用LS3测量系统工作效率*/
                  LS3_ENABLE;       /*低端开关3使能,PTD17=1*/ /*测量系统工作负载率*/
            #endif
            /*================================在本处添加需要的控制函数===================================*/
		/*===========================================================================================*/
	}
}


