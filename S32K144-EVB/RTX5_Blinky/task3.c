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


/*================================================================================�����ļ�ʹ�õĺ궨����===============================================================================*/
/*=====================================================ȫ�ֱ���������===========================================================*/
unsigned int	SoundNumber;						/*����������*/
unsigned int	SoundNumber1;						/*����������*/
/*=====================================================ȫ�ֺ���������===========================================================*/
void  CanCommandVehicle(struct  CAN_MESSAGE_BUFFER *CanData);                 /*����CAN������뺯��*/
void  CanRepeatTranProc(void);                                                /*ѭ������CAN����*/
void  UpdateCanData(struct      CanRepeatTran *);                             /*������Ҫ���͵�CAN����*/
/*-------------------------------------------------------------------------------------------------------------------------------
*                                                      ����3�߳�:
*˵��:����3����������е�CANͨѶ���ܣ��������£�
*     2��SVC������ϵͳ֮�����ģ��֮�������ͨѶ��
*	3��SVC�ϴ��Լ��Ŀ�����Ϣ�������ͼ����λ����
*     4��SVCͨ��CAN�����ϴ������ظ��ֱ궨��Ϣ�����ݡ�
*-------------------------------------------------------------------------------------------------------------------------------*/


void Task3(void *argument)
{
struct  CAN_MESSAGE_BUFFER TempCan;
	(void)argument;
      /*==============================��Ҫ�����õı���ȫ�ֱ���=================================*/
	SoundNumber=0;						/*����������*/
	SoundNumber1=0;						/*����������*/
      /*=======================================================================================*/
	for (;;) 
	{
            #if   SYSTEM_DUTY_ENABLE==1         /*����LS3����ϵͳ����Ч��*/
                  LS3_DISABLE;                  /*�Ͷ˿���3��ֹ,PTD17=0*/
            #endif
		
		StartAtdConver();		/*========================����A/Dת��========================*/
		osDelay(TICK_2mS);	            /*2mSʱ��Ƭ*/		
		GetAtdResult();		/*======================���A/Dת�����======================*/		
            /*=====================================��ȡ����CAN���յ�����==================================================*/
            while(ReadCanData(CanChannel0,&TempCan))      /*��ȡCAN���Ļ�������������*/
		{
			CanCommandVehicle(&TempCan);                 /*����CAN������뺯��*/
		};
            CanRepeatTranProc();          /*ѭ������CAN����*/
	}
}
/*================================================================����CAN������뺯��===========================================================================*/
void  CanCommandVehicle(struct  CAN_MESSAGE_BUFFER *CanData)
{
      if(CanData->IDE)     /*==================================��չ֡ID����====================================*/
      {
						// Լ������չ֡��CAN ID
            switch(CanData->CAN_ID.EXTEND.ID)   /*===========================��չ֡ID����=============================*/
            {
						case 0: /* CAN��չ֡IDΪ0ʱ�Ĵ��� */
            // ִ����Ԥ������
						// task5();
            break;
						case 1: /* CAN��չ֡IDΪ1ʱ�Ĵ��� */
            // ִ���񶯰�Ħ����
						// task6();
            break;
						case 2:
						// ִ�������ɶ�����
						// Task2(NULL);
						break;
			/*=====================================CANAPE�궨����====================================================*/
			case	CCP_CRO_ID:		/* CAN identifier Master -> ECU  :CANAPE--CCP */
				if((HostCommandState & 0XFF0000) == 0)
                        {
						memcpy(CcpDataBuff,CanData->Data,8);	/*��CAN���ݿ�����CCP���ݻ�����*/
						HostCommandState=HostCommandState | MASTER_COMMAND_CANAPE_CALIBRATIOB;        /*��λ����������ʹ��CCP���Э��*/
				}
				break;
			/*=======================================================================================================*/
					
		};
      }else    
      {
            switch(CanData->CAN_ID.STAND.ID)   /*=============================��׼֡ID����=============================*/
            {
			/*=====================================CANAPE�궨����====================================================*/
			case	CCP_CRO_ID:		/* CAN identifier Master -> ECU  :CANAPE--CCP */
				if((HostCommandState & 0XFF0000) == 0)
                              {
						memcpy(CcpDataBuff,CanData->Data,8);	/*��CAN���ݿ�����CCP���ݻ�����*/
						HostCommandState=HostCommandState | MASTER_COMMAND_CANAPE_CALIBRATIOB;        /*��λ����������ʹ��CCP���Э��*/
						//Task2(candata);         
							}
				break;
            };
      }
} 

/*==========================================================================ѭ������CAN����=====================================================================================*/
void  CanRepeatTranProc(void)
{
unsigned int      BuffSize;
struct  CAN_MESSAGE_BUFFER    TempCanData;      /*��ʱ����,CAN����*/
      for(BuffSize=0;BuffSize<CAN_REPEAT_BUFFER_SIZE;BuffSize++)
      {
            if((CanRepeatBuff[BuffSize].RepeatTimes>0) &&(CanRepeatBuff[BuffSize].RepeatTimes<=36000000)) /*ѭ��ʱ��Ƭ������ȷ*/ 
            {
			if(CanRepeatBuff[BuffSize].SurplusTimes>=2)
				CanRepeatBuff[BuffSize].SurplusTimes=CanRepeatBuff[BuffSize].SurplusTimes - 2;
                  if(CanRepeatBuff[BuffSize].SurplusTimes<2)
                  {
                        CanRepeatBuff[BuffSize].SurplusTimes=CanRepeatBuff[BuffSize].RepeatTimes;     /*���¿�ʼ��ʱ*/
                        if(CanRepeatBuff[BuffSize].Times!=0)    /*����CAN����*/     
                        {
                              DISABLE_INTERRUPTS();         /*�ر��������*/
                              if(CanRepeatBuff[BuffSize].Action==0X55)   /*������CAN���ݷ���ǰ��������*/
                                    UpdateCanData(&CanRepeatBuff[BuffSize]);                    /*����CANҪ���͵�����*/
                              ENABLE_INTERRUPTS();          /*ʹ���������*/
                              if(CanRepeatBuff[BuffSize].Times>0) CanRepeatBuff[BuffSize].Times--;
                              TempCanData.IDE=CanRepeatBuff[BuffSize].CanIdType;    /*����֡���ͣ�0����׼֡��1����չ֡*/
                              DISABLE_INTERRUPTS();         /*�ر��������*/
                              if(TempCanData.IDE)
                              {
                                    TempCanData.CAN_ID.EXTEND.ID=CanRepeatBuff[BuffSize].CanID;            /*CANID*/
                                    TempCanData.SRR=1;      /*Ĭ��ֵ:1*/
                              }
                              else  TempCanData.CAN_ID.STAND.ID=CanRepeatBuff[BuffSize].CanID;            /*CANID*/
                              TempCanData.RTR=0;      /*��Զ��֡*/
                              TempCanData.DLC=CanRepeatBuff[BuffSize].DataLength;     /*���ݳ���*/
                              (void)memcpy(TempCanData.Data,CanRepeatBuff[BuffSize].Data,8);    /*��������*/
                              ENABLE_INTERRUPTS();          /*ʹ���������*/
                              (void)CANTranQueue((CANCHANNEL)CanRepeatBuff[BuffSize].CanChannel,&TempCanData);     /*ʹ�ö��з���CAN����*/
                              DISABLE_INTERRUPTS();         /*�ر��������*/
                              if(CanRepeatBuff[BuffSize].Action==0XAA)    /*������CAN���ݷ��ͺ��������*/
                                    UpdateCanData(&CanRepeatBuff[BuffSize]);                    /*����CANҪ���͵�����*/
                              ENABLE_INTERRUPTS();          /*ʹ���������*/
                        }
                  }
            }
      }
}
/*========================================================================================================������Ҫ���͵�CAN����===============================================
      ��CAN��������ǰ����������CAN�������е�����
===============================================================================================================================================================================*/
void  UpdateCanData(struct      CanRepeatTran *CanBuffData) 
{
unsigned	int	TempValue;
      switch(CanBuffData->CanID)
      {
            case	BMU_TO_BCU_CONTROL:       /*BMU�����������ݸ�BCU������BCU����,ID:0X20*/
						CanBuffData->Data[0]=SoundNumber;		/*�����Լ�1*/
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
			CanBuffData->Data[0]=SoundNumber1;		/*�����Լ�1*/
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


