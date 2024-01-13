#include "device_registers.h"           /* Device header */
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "S32K144_API.h"
#include "function.h"
#include "CCP.H"
#include "SystemConfig.h"

extern void                 app_main (void *arg);
extern uint64_t             app_main_stk[];
extern const osThreadAttr_t app_main_attr;
/*========================================================����������===========================================================*/
/*----------------------------------------------------------------------------
  main function
 *---------------------------------------------------------------------------*/
int main(void)
{
      DISABLE_INTERRUPTS();                                       /*�ر��ж�*/
      WATCHDOG_TRIGGER;       /*�������Ź�ιʳ,PTC9��ת*/ 
	SystemCoreClockConfigure();                            	/* configure system core clock */
	SystemCoreClockUpdate();                                	/* update System Core Clock info */
	InitHardware();								/*��ʼ��ϵͳӲ��*/
      WATCHDOG_TRIGGER;       /*�������Ź�ιʳ,PTC9��ת*/ 
      InitVarent();                                               /*��ʼ������ȫ�ֱ���*/
      
      ENABLE_INTERRUPTS();                                        /*���ж�*/
      
	osKernelInitialize();                                   	/* initialize CMSIS-RTOS */
	
      WATCHDOG_TRIGGER;       /*�������Ź�ιʳ,PTC9��ת*/ 
      
      #if   BMS_ECU_CCP_ENABLE ==   1                 		/*ʹ��SVC��CCPЭ�������*/
            ccpInit();              /*CCP��ʼ��*/
      #endif
      
	osThreadNew(app_main, NULL, &app_main_attr);			/* ���������� */
	if (osKernelGetState() == osKernelReady) 
	{
		osKernelStart();                              		/* start thread execution */
	};
	
	for (;;);
}
