#include "device_registers.h"           /* Device header */
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "S32K144_API.h"
#include "function.h"
#include "CCP.H"
#include "SystemConfig.h"

extern void                 app_main (void *arg);
extern uint64_t             app_main_stk[];
extern const osThreadAttr_t app_main_attr;
/*========================================================函数声明区===========================================================*/
/*----------------------------------------------------------------------------
  main function
 *---------------------------------------------------------------------------*/
int main(void)
{
      DISABLE_INTERRUPTS();                                       /*关闭中断*/
      WATCHDOG_TRIGGER;       /*触发看门狗喂食,PTC9翻转*/ 
	SystemCoreClockConfigure();                            	/* configure system core clock */
	SystemCoreClockUpdate();                                	/* update System Core Clock info */
	InitHardware();								/*初始化系统硬件*/
      WATCHDOG_TRIGGER;       /*触发看门狗喂食,PTC9翻转*/ 
      InitVarent();                                               /*初始化所有全局变量*/
      
      ENABLE_INTERRUPTS();                                        /*打开中断*/
      
	osKernelInitialize();                                   	/* initialize CMSIS-RTOS */
	
      WATCHDOG_TRIGGER;       /*触发看门狗喂食,PTC9翻转*/ 
      
      #if   BMS_ECU_CCP_ENABLE ==   1                 		/*使能SVC中CCP协议的驱动*/
            ccpInit();              /*CCP初始化*/
      #endif
      
	osThreadNew(app_main, NULL, &app_main_attr);			/* 生成主任务 */
	if (osKernelGetState() == osKernelReady) 
	{
		osKernelStart();                              		/* start thread execution */
	};
	
	for (;;);
}
