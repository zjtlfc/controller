#include <stdio.h>
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "RTE_Components.h"             /* Component selection */
#include "Task.h"
#include "S32K144_API.h"
#include "S32K144.h"
#include "SystemConfig.h"
#include "function.h"
#include "s32_core_cm4.h"
#include  "Bms-Input-Output-Port.h"
#include "string.h"

/*===================================================本文件使用的全局变量======================================================*/
/*===================================================本文家使用的函数声明======================================================*/


/*-------------------------------------------------------------------------------------------------------------------------------
*                                                      任务6线程:
*说明:任务6用来完成所有与诊断相关的功能。
*-------------------------------------------------------------------------------------------------------------------------------*/
void Task6(void *argument) 
{
	(void)argument;
      /*=============================================变量初始化================================================*/
      /*=======================================================================================================*/
	for (;;) 
	{
            /*=========================================================================================*/
		osDelay(TICK_10mS);	/*10mS时间片*/
            /*================================在本处添加需要的控制函数===================================*/
		/*===========================================================================================*/
	}
};



