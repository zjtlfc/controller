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
/*=====================================================���ļ�ʹ�õĺ궨��========================================================*/
/*===============================================================�������������==================================================*/
uint64_t app_main_stk[APP_MAIN_STK_SZ / 8];	      /*�������ջ1024*/
const osThreadAttr_t app_main_attr = 
{
	.stack_mem  = &app_main_stk[0],		      /*�������ջָ��*/
	.stack_size = sizeof(app_main_stk),		      /*�������ջ�ߴ�*/
	.priority=MAINTASK_priority			      /*���������ȼ�*/
};
/*===============================================================����1��������==================================================*/
uint64_t app_Task1_stk[APP_Task1_STK_SZ / 8];	      /*����1��ջ512�ֽ�*/
const osThreadAttr_t app_Task1_attr = 
{
	.stack_mem  = &app_Task1_stk[0],		      /*����1���ջָ��*/
	.stack_size = sizeof(app_Task1_stk),	      /*����1���ջ�ߴ�*/
	.priority=TASK1_priority			      /*����1�����ȼ�*/
};
/*===============================================================����2��������==================================================*/
uint64_t app_Task2_stk[APP_Task2_STK_SZ / 8];	      /*����2��ջ512�ֽ�*/
const osThreadAttr_t app_Task2_attr = 
{
	.stack_mem  = &app_Task2_stk[0],		      /*����2��ջָ��*/
	.stack_size = sizeof(app_Task2_stk),	      /*����2��ջ�ߴ�*/
	.priority=TASK2_priority			      /*����2���ȼ�*/
};
/*===============================================================����3��������==================================================*/
uint64_t app_Task3_stk[APP_Task3_STK_SZ / 8];	      /*����3��ջ512�ֽ�*/
const osThreadAttr_t app_Task3_attr = 
{
	.stack_mem  = &app_Task3_stk[0],		      /*����3��ջָ��*/
	.stack_size = sizeof(app_Task3_stk),	      /*����3��ջ�ߴ�*/
	.priority=TASK3_priority			      /*����3���ȼ�*/
};
/*===============================================================����4��������==================================================*/
uint64_t app_Task4_stk[APP_Task4_STK_SZ / 8];	      /*����4��ջ512�ֽ�*/
const osThreadAttr_t app_Task4_attr = 
{
	.stack_mem  = &app_Task4_stk[0],		      /*����4��ջָ��*/
	.stack_size = sizeof(app_Task4_stk),	      /*����4��ջ�ߴ�*/
	.priority=TASK4_priority			      /*����4���ȼ�*/
};
/*====================================================ȫ�ֱ�������==============================================================*/
osThreadId_t tid_Task1;                		      /* ����1�߳̾�� */
osThreadId_t tid_Task2;                		      /* ����2�߳̾�� */
osThreadId_t tid_Task3;                		      /* ����3�߳̾�� */
osThreadId_t tid_Task4;                		      /* ����4�߳̾�� */
/*====================================================ȫ�ֺ�������==============================================================*/
/*===================================================��������===================================================================*/
extern 	void Task1(void *argument);		      /* ����1*/
extern 	void Task2(void *argument);		      /* ����2*/
extern 	void Task3(void *argument);		      /* ����3*/
extern 	void Task4(void *argument);		      /* ����4*/
//extern 	void Task5(void *argument);		      /* ����5*/
//extern 	void Task6(void *argument);		      /* ����6*/
/*-------------------------------------------------------------------------------------------------------------------------------
*                                                      �������߳�:
*˵��:������������ϵͳ�����Ժ����״̬�����ƣ���Ҫ������¼�������
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
	/*=================================================��������ϵͳ�������===========================================*/
	tid_Task1 = osThreadNew (Task1, NULL, &app_Task1_attr);            /* ��������1 */
	if (tid_Task1 == NULL) { /* add error handling */ }
	tid_Task2 = osThreadNew (Task2, NULL, &app_Task2_attr);            /* ��������2 */
	if (tid_Task2 == NULL) { /* add error handling */ }
	tid_Task3 = osThreadNew (Task3, NULL, &app_Task3_attr);            /* ��������3 */
	if (tid_Task3 == NULL) { /* add error handling */ }
	tid_Task4 = osThreadNew (Task4, NULL, &app_Task4_attr);            /* ��������4 */
	if (tid_Task4 == NULL) { /* add error handling */ }
	/*=============================================��������忪ʼִ��===============================================*/
      /*================================================�ֲ���������ֵ==================================================*/
      /*================================================================================================================*/
	for (;;) 
	{
            /*===========================ϵͳʱ���ۼ�============================================*/
            SysTickTime=osKernelGetTickCount() / ( OS_TICK_FREQ / 100) ;      /*ϵͳ������ʱ��,����10mS*/  
            SystemTimes = SysTickTime / 100;    /*��ȡϵͳ�ϵ�������ʱ��,��λS*/
            /*===================================================================================*/
            #if   SYSTEM_DUTY_ENABLE==1                 /*����LS3����ϵͳ����Ч��*/
                  LS3_DISABLE;            /*�Ͷ˿���3��ֹ,PTD17=0*/  /*����ϵͳ����������*/
            #endif
            /*==================================================================================*/
		osDelay(TICK_20mS);	/*20mSʱ��Ƭ*/
            /*==================================================================================*/
            #if   SYSTEM_DUTY_ENABLE==1                 /*����LS3����ϵͳ����Ч��*/
                  LS3_ENABLE;       /*�Ͷ˿���3ʹ��,PTD17=1*/ /*����ϵͳ����������*/
            #endif
            /*================================�ڱ��������Ҫ�Ŀ��ƺ���===================================*/
		/*===========================================================================================*/
	}
}


