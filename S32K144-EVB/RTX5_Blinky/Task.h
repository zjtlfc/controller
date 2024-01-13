#include "stdint.h"
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

/*===========================================================所有任务堆栈尺寸定义================================================*/
#define 	APP_MAIN_STK_SZ 			(1024U)		/*主任务堆栈512字节*/
#define 	APP_Task1_STK_SZ 			(1024U)		/*任务1堆栈512字节*/
#define 	APP_Task2_STK_SZ 			(1024U)		/*任务2堆栈512字节*/
#define 	APP_Task3_STK_SZ 			(1024U)		/*任务3堆栈512字节*/
#define 	APP_Task4_STK_SZ 			(1024U)		/*任务4堆栈512字节*/
/*===========================================所有任务优先级定义(数值越大优先级越高) ==============================================*/
#define	TASK1_priority			osPriorityHigh6		/*Task1任务的优先级：43（高+6）*/
#define	TASK2_priority			osPriorityHigh2		/*Task1任务的优先级：42（高+2）*/
#define	TASK3_priority			osPriorityAboveNormal6	/*Task1任务的优先级：38（偏高+6）*/
#define	MAINTASK_priority			osPriorityAboveNormal2	/*Task1任务的优先级：34（偏高+2）*/
#define	TASK4_priority			osPriorityNormal6		/*Task1任务的优先级：30（正常+6）*/
/*======================================================定义所有的时间片常熟=====================================================*/
#define	TICK_2mS				1				/*2mS时间片*/
#define	TICK_4mS				2				/*4mS时间片*/
#define	TICK_8mS				8				/*8mS时间片*/
#define	TICK_10mS				5				/*10mS时间片*/
#define	TICK_14mS				7				/*14mS时间片*/
#define	TICK_16mS				8				/*16mS时间片*/
#define	TICK_20mS				10				/*20mS时间片*/
#define	TICK_24mS				12				/*25mS时间片*/
#define	TICK_30mS				15				/*30mS时间片*/
#define	TICK_40mS				20				/*40mS时间片*/
#define	TICK_50mS				25				/*50mS时间片*/
#define	TICK_100mS				50				/*100mS时间片*/
#define	TICK_200mS				100				/*200mS时间片*/
#define	TICK_400mS				200				/*400mS时间片*/
#define	TICK_500mS				250				/*500mS时间片*/
#define	TICK_1000mS				500				/*1S时间片*/
#define	TICK_2000mS				1000				/*2S时间片*/
#define	TICK_4000mS				2000				/*4S时间片*/
#define	TICK_5000mS				2500				/*5S时间片*/
#define	TICK_10S				5000				/*10S时间片*/
/*=============================================================全局变量声明======================================================*/

/*===============================================================任务声明========================================================*/
extern	osThreadId_t tid_Task1;           		     /* Thread id of thread: ADC */
extern	osThreadId_t tid_Task2;                		/* Thread id of thread: LED */
extern	osThreadId_t tid_Task3;                		/* Thread id of thread: BUT */
extern	osThreadId_t tid_Task4;                		/* Thread id of thread: BUT */
