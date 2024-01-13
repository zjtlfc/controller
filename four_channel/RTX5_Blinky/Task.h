#include "stdint.h"
#include "cmsis_os2.h"                  // ::CMSIS:RTOS2

/*===========================================================���������ջ�ߴ綨��================================================*/
#define 	APP_MAIN_STK_SZ 			(1024U)		/*�������ջ512�ֽ�*/
#define 	APP_Task1_STK_SZ 			(1024U)		/*����1��ջ512�ֽ�*/
#define 	APP_Task2_STK_SZ 			(1024U)		/*����2��ջ512�ֽ�*/
#define 	APP_Task3_STK_SZ 			(1024U)		/*����3��ջ512�ֽ�*/
#define 	APP_Task4_STK_SZ 			(1024U)		/*����4��ջ512�ֽ�*/
/*===========================================�����������ȼ�����(��ֵԽ�����ȼ�Խ��) ==============================================*/
#define	TASK1_priority			osPriorityHigh6		/*Task1��������ȼ���43����+6��*/
#define	TASK2_priority			osPriorityHigh2		/*Task1��������ȼ���42����+2��*/
#define	TASK3_priority			osPriorityAboveNormal6	/*Task1��������ȼ���38��ƫ��+6��*/
#define	MAINTASK_priority			osPriorityAboveNormal2	/*Task1��������ȼ���34��ƫ��+2��*/
#define	TASK4_priority			osPriorityNormal6		/*Task1��������ȼ���30������+6��*/
/*======================================================�������е�ʱ��Ƭ����=====================================================*/
#define	TICK_2mS				1				/*2mSʱ��Ƭ*/
#define	TICK_4mS				2				/*4mSʱ��Ƭ*/
#define	TICK_8mS				8				/*8mSʱ��Ƭ*/
#define	TICK_10mS				5				/*10mSʱ��Ƭ*/
#define	TICK_14mS				7				/*14mSʱ��Ƭ*/
#define	TICK_16mS				8				/*16mSʱ��Ƭ*/
#define	TICK_20mS				10				/*20mSʱ��Ƭ*/
#define	TICK_24mS				12				/*25mSʱ��Ƭ*/
#define	TICK_30mS				15				/*30mSʱ��Ƭ*/
#define	TICK_40mS				20				/*40mSʱ��Ƭ*/
#define	TICK_50mS				25				/*50mSʱ��Ƭ*/
#define	TICK_100mS				50				/*100mSʱ��Ƭ*/
#define	TICK_200mS				100				/*200mSʱ��Ƭ*/
#define	TICK_400mS				200				/*400mSʱ��Ƭ*/
#define	TICK_500mS				250				/*500mSʱ��Ƭ*/
#define	TICK_1000mS				500				/*1Sʱ��Ƭ*/
#define	TICK_2000mS				1000				/*2Sʱ��Ƭ*/
#define	TICK_4000mS				2000				/*4Sʱ��Ƭ*/
#define	TICK_5000mS				2500				/*5Sʱ��Ƭ*/
#define	TICK_10S				5000				/*10Sʱ��Ƭ*/
/*=============================================================ȫ�ֱ�������======================================================*/

/*===============================================================��������========================================================*/
extern	osThreadId_t tid_Task1;           		     /* Thread id of thread: ADC */
extern	osThreadId_t tid_Task2;                		/* Thread id of thread: LED */
extern	osThreadId_t tid_Task3;                		/* Thread id of thread: BUT */
extern	osThreadId_t tid_Task4;                		/* Thread id of thread: BUT */
