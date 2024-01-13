#ifndef _TASK6_H_
#define _TASK6_H_


#include <stdio.h>
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "RTE_Components.h"             /* Component selection */
#include "Task.h"
#include "S32K144.h"
#include "S32K144_API.h"
#include "SystemConfig.h"
#include "function.h"
#include "s32_core_cm4.h"
#include "Bms-Input-Output-Port.h"
#include "string.h"
#include "arm_math.h"


static int VibrationMassage_threshold = 0;

void Task6(void *argument) 
{
unsigned	int	TempSize;
	(void)argument;


	for (;;) 
	{

		DISABLE_FS0_GR2;		
		osDelay(TICK_16mS);	
		ENABLE_FS0_GR2;		
   		struct  CAN_MESSAGE_BUFFER TempCan2;
		ReadCanData(CanChannel0,&TempCan2);
		if (TempCan2.Data[0] != 0) {
			VibrationMassage_threshold = 1;
		} else {
			VibrationMassage_threshold = 0;
		}
		

		}		
		#endif
		/*===============================================================================================================================================================*/
		(void)OutputLRData(SoundLeftOutData,SoundRightOutData,SoundLeftOutData1,SoundRightOutData1,TempSize);	/*????????????????????????????????????????*/
		/*================================================================================================*/		
		}
	




