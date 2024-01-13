/*
 * Copyright (c) 2013-2017 ARM Limited. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * -----------------------------------------------------------------------------
 *
 * $Revision:   V5.1.0
 *
 * Project:     CMSIS-RTOS RTX
 * Title:       RTX Configuration
 *
 * -----------------------------------------------------------------------------
 */
 
#include "cmsis_compiler.h"
#include "rtx_os.h"
#include "S32K144_API.h"
#include "S32K144.h"
#include "SystemConfig.h"
#include "function.h"
#include "s32_core_cm4.h"
#include "CCP.H"
#include "RTX_Config.h"
// OS Idle Thread
__WEAK __NO_RETURN void osRtxIdleThread (void *argument) {
  (void)argument;
int TempPoint;
unsigned    int   TempTimeTicks;
	for (;;) 
	{
            /*================================��λ���������===========================================*/
            #if   HOST_COMMAND_RUN_IDLE_TASK == 1                         /*��λ���������IDLE������ִ��*/ 
                  if((HostCommandState & 0XFF) == MASTER_COMMAND_PARAMETER_TRANS)               /*��������ϵͳ*/
                  {
//                        ParameterTransProc(HostData);             /*��������ϵͳ�����*/
                        HostCommandState=HostCommandState & 0XFFFFFF00;       /*��������־*/
                  };
                  if((HostCommandState & 0XFF00) == MASTER_COMMAND_VARENT_UPLOAD)               /*������¼������λ��*/
                  {
//                        VarmeteruUploadProc(HostData);            /*������¼�ϴ�ϵͳ����*/
                        HostCommandState=HostCommandState & 0XFFFF00FF;       /*��������־*/
                  };
            #endif
            #if   BMS_ECU_CCP_ENABLE ==   1                 /*ʹ��BMS��CCPЭ�������*/
                  if((HostCommandState & 0XFF0000) == MASTER_COMMAND_CANAPE_CALIBRATIOB)        /*��λ��CANAPE����ִ��*/
                  {
                        ccpCommand( CcpDataBuff);     /*������Ӧ��CCP�������*/
                        HostCommandState=HostCommandState & 0XFF00FFFF;       /*��������־*/
                  };
                  TempTimeTicks=osKernelGetTickCount();
                  if(SystemTicksPrev != TempTimeTicks)      /*���ʱ�ӽ��ķ����仯*/
                  {
                        SystemTicksPrev=TempTimeTicks;
                       if((TempTimeTicks % (20 * OS_TICK_FREQ / 1000)) == 1)  	ccpDaq(0);   	/*20mS DAQʹ��ͨ��0 */
                        if((TempTimeTicks % (50 * OS_TICK_FREQ / 1000)) == 2)    	ccpDaq(1);   	/*50mS DAQʹ��ͨ��1 */
                        if((TempTimeTicks % (100 * OS_TICK_FREQ / 1000)) == 3)   	ccpDaq(2);   	/*100mS DAQʹ��ͨ��2 */
                        if((TempTimeTicks % (200 * OS_TICK_FREQ / 1000)) == 4)   	ccpDaq(3);    	/*200mS DAQʹ��ͨ��3 */
                  };
            #endif
            #if   BMS_UDS_DIAGON_ENABLE  == 1                 /*ʹ��BMS��UDS��Ϲ���*/
                  if((HostCommandState & 0XFF000000) == MASTER_COMMAND_UDS_DIAGONSE)            /*��λ�����UDS����ִ��*/
                  {
                        HostCommandState=HostCommandState & 0X00FFFFFF;       /*��������־*/
                  };
            #endif
            /*====================================CAN���з������ݹ���================================*/
            if(Can0TranTop != Can0TranTail)       /*CAN0���ͻ������д���������*/
            {
                  DISABLE_INTERRUPTS();         /*�ر��������*/
                  TempPoint=Can0TranTop+1;
                  if(TempPoint>=CAN0_TRAN_BUFF_SIZE)  TempPoint=0;
                  if(TranCan(CanChannel0,&Can0TranBuffer[TempPoint])==0)      /*�������뷢�ͻ�����*/
                        Can0TranTop=TempPoint;
                  ENABLE_INTERRUPTS();          /*ʹ���������*/
            };
            if(Can1TranTop != Can1TranTail)       /*CAN1���ͻ������д���������*/
            {
                  DISABLE_INTERRUPTS();         /*�ر��������*/
                  TempPoint=Can1TranTop+1;
                  if(TempPoint>=CAN1_TRAN_BUFF_SIZE)  TempPoint=0;
                  if(TranCan(CanChannel1,&Can1TranBuffer[TempPoint])==0)      /*�������뷢�ͻ�����*/
                        Can1TranTop=TempPoint;
                  ENABLE_INTERRUPTS();          /*ʹ���������*/
            };
            if(Can2TranTop != Can2TranTail)       /*CAN2���ͻ������д���������*/
            {
                  DISABLE_INTERRUPTS();         /*�ر��������*/
                  TempPoint=Can2TranTop+1;
                  if(TempPoint>=CAN2_TRAN_BUFF_SIZE)  TempPoint=0;
                  if(TranCan(CanChannel2,&Can2TranBuffer[TempPoint])==0)       /*�������뷢�ͻ�����*/
                        Can2TranTop=TempPoint;
                  ENABLE_INTERRUPTS();          /*ʹ���������*/
            };
            /*=======================================================================================*/
	}
}
 
// OS Error Callback function
__WEAK uint32_t osRtxErrorNotify (uint32_t code, void *object_id) {
  (void)object_id;

  switch (code) {
    case osRtxErrorStackUnderflow:
      // Stack overflow detected for thread (thread_id=object_id)
      break;
    case osRtxErrorISRQueueOverflow:
      // ISR Queue overflow detected when inserting object (object_id)
      break;
    case osRtxErrorTimerQueueOverflow:
      // User Timer Callback Queue overflow detected for timer (timer_id=object_id)
      break;
    case osRtxErrorClibSpace:
      // Standard C/C++ library libspace not available: increase OS_THREAD_LIBSPACE_NUM
      break;
    case osRtxErrorClibMutex:
      // Standard C/C++ library mutex initialization failed
      break;
    default:
      break;
  }
  for (;;) {}
//return 0U;
}
