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
            /*================================上位机命令解析===========================================*/
            #if   HOST_COMMAND_RUN_IDLE_TASK == 1                         /*上位机命令函数在IDLE任务中执行*/ 
                  if((HostCommandState & 0XFF) == MASTER_COMMAND_PARAMETER_TRANS)               /*参数传输系统*/
                  {
//                        ParameterTransProc(HostData);             /*参数传输系统命令函数*/
                        HostCommandState=HostCommandState & 0XFFFFFF00;       /*清除命令标志*/
                  };
                  if((HostCommandState & 0XFF00) == MASTER_COMMAND_VARENT_UPLOAD)               /*变量记录上载上位机*/
                  {
//                        VarmeteruUploadProc(HostData);            /*变量记录上传系统函数*/
                        HostCommandState=HostCommandState & 0XFFFF00FF;       /*清除命令标志*/
                  };
            #endif
            #if   BMS_ECU_CCP_ENABLE ==   1                 /*使能BMS中CCP协议的驱动*/
                  if((HostCommandState & 0XFF0000) == MASTER_COMMAND_CANAPE_CALIBRATIOB)        /*上位机CANAPE命令执行*/
                  {
                        ccpCommand( CcpDataBuff);     /*运行相应的CCP命令解码*/
                        HostCommandState=HostCommandState & 0XFF00FFFF;       /*清除命令标志*/
                  };
                  TempTimeTicks=osKernelGetTickCount();
                  if(SystemTicksPrev != TempTimeTicks)      /*如果时钟节拍发生变化*/
                  {
                        SystemTicksPrev=TempTimeTicks;
                       if((TempTimeTicks % (20 * OS_TICK_FREQ / 1000)) == 1)  	ccpDaq(0);   	/*20mS DAQ使用通道0 */
                        if((TempTimeTicks % (50 * OS_TICK_FREQ / 1000)) == 2)    	ccpDaq(1);   	/*50mS DAQ使用通道1 */
                        if((TempTimeTicks % (100 * OS_TICK_FREQ / 1000)) == 3)   	ccpDaq(2);   	/*100mS DAQ使用通道2 */
                        if((TempTimeTicks % (200 * OS_TICK_FREQ / 1000)) == 4)   	ccpDaq(3);    	/*200mS DAQ使用通道3 */
                  };
            #endif
            #if   BMS_UDS_DIAGON_ENABLE  == 1                 /*使能BMS的UDS诊断功能*/
                  if((HostCommandState & 0XFF000000) == MASTER_COMMAND_UDS_DIAGONSE)            /*上位机诊断UDS命令执行*/
                  {
                        HostCommandState=HostCommandState & 0X00FFFFFF;       /*清除命令标志*/
                  };
            #endif
            /*====================================CAN队列发送数据过程================================*/
            if(Can0TranTop != Can0TranTail)       /*CAN0发送缓冲区有待发送数据*/
            {
                  DISABLE_INTERRUPTS();         /*关闭任务调度*/
                  TempPoint=Can0TranTop+1;
                  if(TempPoint>=CAN0_TRAN_BUFF_SIZE)  TempPoint=0;
                  if(TranCan(CanChannel0,&Can0TranBuffer[TempPoint])==0)      /*数据送入发送缓冲区*/
                        Can0TranTop=TempPoint;
                  ENABLE_INTERRUPTS();          /*使能任务调度*/
            };
            if(Can1TranTop != Can1TranTail)       /*CAN1发送缓冲区有待发送数据*/
            {
                  DISABLE_INTERRUPTS();         /*关闭任务调度*/
                  TempPoint=Can1TranTop+1;
                  if(TempPoint>=CAN1_TRAN_BUFF_SIZE)  TempPoint=0;
                  if(TranCan(CanChannel1,&Can1TranBuffer[TempPoint])==0)      /*数据送入发送缓冲区*/
                        Can1TranTop=TempPoint;
                  ENABLE_INTERRUPTS();          /*使能任务调度*/
            };
            if(Can2TranTop != Can2TranTail)       /*CAN2发送缓冲区有待发送数据*/
            {
                  DISABLE_INTERRUPTS();         /*关闭任务调度*/
                  TempPoint=Can2TranTop+1;
                  if(TempPoint>=CAN2_TRAN_BUFF_SIZE)  TempPoint=0;
                  if(TranCan(CanChannel2,&Can2TranBuffer[TempPoint])==0)       /*数据送入发送缓冲区*/
                        Can2TranTop=TempPoint;
                  ENABLE_INTERRUPTS();          /*使能任务调度*/
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
