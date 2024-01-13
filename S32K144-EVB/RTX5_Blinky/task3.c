#ifndef _TASK3_H_
#define _TASK3_H_

#include <stdio.h>
#include "cmsis_os2.h"                  /* ::CMSIS:RTOS2 */
#include "RTE_Components.h"             /* Component selection */
#include "Task.h"
#include "S32K144.h"
#include "S32K144_API.h"
#include "SystemConfig.h"
#include "string.h"
#include "function.h"
#include "s32_core_cm4.h"
#include  "Bms-Input-Output-Port.h"
#include  "CCPPAR.h"


extern 	void Task2(void *argument);


/*================================================================================本地文件使用的宏定义区===============================================================================*/
/*=====================================================全局变量定义区===========================================================*/
unsigned int	SoundNumber;						/*测试用数据*/
unsigned int	SoundNumber1;						/*测试用数据*/
/*=====================================================全局函数声明区===========================================================*/
void  CanCommandVehicle(struct  CAN_MESSAGE_BUFFER *CanData);                 /*整车CAN命令解码函数*/
void  CanRepeatTranProc(void);                                                /*循环发送CAN数据*/
void  UpdateCanData(struct      CanRepeatTran *);                             /*更新想要发送的CAN数据*/
/*-------------------------------------------------------------------------------------------------------------------------------
*                                                      任务3线程:
*说明:任务3用来完成所有的CAN通讯功能，具体如下：
*     2、SVC与整车系统之间各个模块之间的数据通讯。
*	3、SVC上传自己的控制信息给整车和监控上位机。
*     4、SVC通过CAN总线上传、下载各种标定信息和数据。
*-------------------------------------------------------------------------------------------------------------------------------*/


void Task3(void *argument)
{
struct  CAN_MESSAGE_BUFFER TempCan;
	(void)argument;
      /*==============================需要赋处置的本地全局变量=================================*/
	SoundNumber=0;						/*测试用数据*/
	SoundNumber1=0;						/*测试用数据*/
      /*=======================================================================================*/
	for (;;) 
	{
            #if   SYSTEM_DUTY_ENABLE==1         /*启用LS3测量系统工作效率*/
                  LS3_DISABLE;                  /*低端开关3禁止,PTD17=0*/
            #endif
		
		StartAtdConver();		/*========================启动A/D转换========================*/
		osDelay(TICK_2mS);	            /*2mS时间片*/		
		GetAtdResult();		/*======================获得A/D转换结果======================*/		
            /*=====================================读取整车CAN接收的数据==================================================*/
            while(ReadCanData(CanChannel0,&TempCan))      /*读取CAN报文缓冲区接收数据*/
		{
			CanCommandVehicle(&TempCan);                 /*整车CAN命令解码函数*/
		};
            CanRepeatTranProc();          /*循环发送CAN数据*/
	}
}
/*================================================================整车CAN命令解码函数===========================================================================*/
void  CanCommandVehicle(struct  CAN_MESSAGE_BUFFER *CanData)
{
      if(CanData->IDE)     /*==================================扩展帧ID处理====================================*/
      {
						// 约定好拓展帧的CAN ID
            switch(CanData->CAN_ID.EXTEND.ID)   /*===========================扩展帧ID处理=============================*/
            {
						case 0: /* CAN扩展帧ID为0时的处理 */
            // 执行振动预警函数
						// task5();
            break;
						case 1: /* CAN扩展帧ID为1时的处理 */
            // 执行振动按摩函数
						// task6();
            break;
						case 2:
						// 执行音乐律动函数
						// Task2(NULL);
						break;
			/*=====================================CANAPE标定数据====================================================*/
			case	CCP_CRO_ID:		/* CAN identifier Master -> ECU  :CANAPE--CCP */
				if((HostCommandState & 0XFF0000) == 0)
                        {
						memcpy(CcpDataBuff,CanData->Data,8);	/*将CAN数据拷贝到CCP数据缓冲区*/
						HostCommandState=HostCommandState | MASTER_COMMAND_CANAPE_CALIBRATIOB;        /*上位机发送命令使用CCP诊断协议*/
				}
				break;
			/*=======================================================================================================*/
					
		};
      }else    
      {
            switch(CanData->CAN_ID.STAND.ID)   /*=============================标准帧ID处理=============================*/
            {
			/*=====================================CANAPE标定数据====================================================*/
			case	CCP_CRO_ID:		/* CAN identifier Master -> ECU  :CANAPE--CCP */
				if((HostCommandState & 0XFF0000) == 0)
                              {
						memcpy(CcpDataBuff,CanData->Data,8);	/*将CAN数据拷贝到CCP数据缓冲区*/
						HostCommandState=HostCommandState | MASTER_COMMAND_CANAPE_CALIBRATIOB;        /*上位机发送命令使用CCP诊断协议*/
						//Task2(candata);         
							}
				break;
            };
      }
} 

/*==========================================================================循环发送CAN数据=====================================================================================*/
void  CanRepeatTranProc(void)
{
unsigned int      BuffSize;
struct  CAN_MESSAGE_BUFFER    TempCanData;      /*临时变量,CAN数据*/
      for(BuffSize=0;BuffSize<CAN_REPEAT_BUFFER_SIZE;BuffSize++)
      {
            if((CanRepeatBuff[BuffSize].RepeatTimes>0) &&(CanRepeatBuff[BuffSize].RepeatTimes<=36000000)) /*循环时间片设置正确*/ 
            {
			if(CanRepeatBuff[BuffSize].SurplusTimes>=2)
				CanRepeatBuff[BuffSize].SurplusTimes=CanRepeatBuff[BuffSize].SurplusTimes - 2;
                  if(CanRepeatBuff[BuffSize].SurplusTimes<2)
                  {
                        CanRepeatBuff[BuffSize].SurplusTimes=CanRepeatBuff[BuffSize].RepeatTimes;     /*重新开始计时*/
                        if(CanRepeatBuff[BuffSize].Times!=0)    /*发送CAN数据*/     
                        {
                              DISABLE_INTERRUPTS();         /*关闭任务调度*/
                              if(CanRepeatBuff[BuffSize].Action==0X55)   /*允许在CAN数据发送前更新数据*/
                                    UpdateCanData(&CanRepeatBuff[BuffSize]);                    /*更新CAN要发送的数据*/
                              ENABLE_INTERRUPTS();          /*使能任务调度*/
                              if(CanRepeatBuff[BuffSize].Times>0) CanRepeatBuff[BuffSize].Times--;
                              TempCanData.IDE=CanRepeatBuff[BuffSize].CanIdType;    /*报文帧类型，0：标准帧；1：扩展帧*/
                              DISABLE_INTERRUPTS();         /*关闭任务调度*/
                              if(TempCanData.IDE)
                              {
                                    TempCanData.CAN_ID.EXTEND.ID=CanRepeatBuff[BuffSize].CanID;            /*CANID*/
                                    TempCanData.SRR=1;      /*默认值:1*/
                              }
                              else  TempCanData.CAN_ID.STAND.ID=CanRepeatBuff[BuffSize].CanID;            /*CANID*/
                              TempCanData.RTR=0;      /*非远程帧*/
                              TempCanData.DLC=CanRepeatBuff[BuffSize].DataLength;     /*数据长度*/
                              (void)memcpy(TempCanData.Data,CanRepeatBuff[BuffSize].Data,8);    /*复制数据*/
                              ENABLE_INTERRUPTS();          /*使能任务调度*/
                              (void)CANTranQueue((CANCHANNEL)CanRepeatBuff[BuffSize].CanChannel,&TempCanData);     /*使用队列发送CAN数据*/
                              DISABLE_INTERRUPTS();         /*关闭任务调度*/
                              if(CanRepeatBuff[BuffSize].Action==0XAA)    /*允许在CAN数据发送后更新数据*/
                                    UpdateCanData(&CanRepeatBuff[BuffSize]);                    /*更新CAN要发送的数据*/
                              ENABLE_INTERRUPTS();          /*使能任务调度*/
                        }
                  }
            }
      }
}
/*========================================================================================================更新想要发送的CAN数据===============================================
      在CAN发送数据前后用来更新CAN缓冲区中的数据
===============================================================================================================================================================================*/
void  UpdateCanData(struct      CanRepeatTran *CanBuffData) 
{
unsigned	int	TempValue;
      switch(CanBuffData->CanID)
      {
            case	BMU_TO_BCU_CONTROL:       /*BMU发送命令数据给BCU，控制BCU运行,ID:0X20*/
						CanBuffData->Data[0]=SoundNumber;		/*数据自加1*/
			CanBuffData->Data[1]=SoundNumber >> 8;		
			TempValue=SoundOutputCH3[SoundNumber];
			CanBuffData->Data[2]=TempValue;		
			CanBuffData->Data[3]=TempValue >> 8;
			TempValue=SoundOutputCH4[SoundNumber];		
			CanBuffData->Data[4]=TempValue;		
			CanBuffData->Data[5]=TempValue >> 8;		
			CanBuffData->Data[6]=ReceDataSize;		
			CanBuffData->Data[7]=ReceDataSize >> 8;		
			SoundNumber++;
			if(SoundNumber >= IIS_RECE_BUFF_SIZE) SoundNumber=0;
			break;
		case	BMS_RUN_MESSAGE:
			CanBuffData->Data[0]=SoundNumber1;		/*数据自加1*/
			CanBuffData->Data[1]=SoundNumber1 >> 8;		
			CanBuffData->Data[2]=0;		
			CanBuffData->Data[3]=0;		
			TempValue=IisDataBuffer[SoundNumber1];
			CanBuffData->Data[4]=TempValue;		
			CanBuffData->Data[5]=TempValue >> 8;
			CanBuffData->Data[6]=TempValue>> 16;		
			CanBuffData->Data[7]=TempValue >> 24;		
			SoundNumber1++;
			if(SoundNumber1 >= IIS_RECE_BUFF_SIZE) SoundNumber1=0;
			break;
      };
};

#endif


