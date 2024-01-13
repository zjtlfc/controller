#include "S32K144_API.h"
#include "system_S32K144.h"             // Keil::Device:Startup
#include "clock_S32K1xx.h"              // Keil::Device:SDK Driver:Clock
#include "clock_manager.h"              // Keil::Device:SDK Driver:Clock
#include "string.h"
//#include "S32K144.h"
#include "function.h"
#include "SystemConfig.h"
#include "s32_core_cm4.h"
/*====================================================================�궨����==============================================================*/
/*===================================================================����������=============================================================*/
/*=================================================================ȫ�ֱ���������===========================================================*/
struct      CanRepeatTran     CanRepeatBuff[CAN_REPEAT_BUFFER_SIZE]=                /*CANѭ�����ͻ���������*/
      {     /*   ����  ��ǰʱ��     CAN_ID           ���ݳ��� ���� ID����  ͨ��     ��������     ����ֵ    */
                  10,   10,   BMU_TO_BCU_CONTROL,          8, 0XAA,  0,  SYSTEM_CAN, -1,       {0X0C,1,0,0,0,0,0,0},                 /*��BCU�������� ��о��ѹ���¶�ID:0X20*/
                  10,   10,   BMS_RUN_MESSAGE,             8, 0X55,  1,  SYSTEM_CAN, -1,       {0,0,0,0,0,0,0,0},                    /*BMSϵͳ����״̬��Ϣ,ID=0X012D11EB(19730923)*/
                  100,  100,  CELL_STATUS_MESSAGE,         8, 0X55,  1,  SYSTEM_CAN, 0,       {0,0,0,0,0,0,0,0},                    /*��ذ���о״̬��Ϣ,ID=0X012DFBE8(19790824)*/
                  500, 500,   BMS_AUX1_MESSAGE,            8, 0X55,  1,  SYSTEM_CAN, 0,       {0,0,0,0,0,0,0,0},                    /*BMS���и�����Ϣ,ID=0X013218FA(20060410)*/
      };          
unsigned 	int	SysTickTime;		/*ϵͳ������ʱ��,����10mS*/  
unsigned 	int	SystemTimes;    		/*��ȡϵͳ�ϵ�������ʱ��,��λS*/
unsigned   int	IisDataBuffer[IIS_RECE_BUFF_SIZE+5];				/*IIS���ݽ��ջ�����*/
unsigned	int	IisReceTop,IisReceBottom;					/*IIS�������ݻ�������βָ��*/
unsigned	int	ReceDataSize;							/*����Ϊ�������ݳ���*/
			
unsigned	short		SoundOutputCH1[SOUND_DRIVE_BUFF_SIZE+5];	/*�������1�����ݣ�0��1666��*/
unsigned	short		SoundOutputCH2[SOUND_DRIVE_BUFF_SIZE+5];	/*�������2�����ݣ�0��1666��*/	
unsigned	short		SoundOutputCH3[SOUND_DRIVE_BUFF_SIZE+5];	/*�������3�����ݣ�0��1666��*/
unsigned	short		SoundOutputCH4[SOUND_DRIVE_BUFF_SIZE+5];	/*�������4�����ݣ�0��1666��*/	
			
unsigned	short	IisLeftSoundData[SOUND_DATA_BUFF_SIZE],IisRightSoundData[SOUND_DATA_BUFF_SIZE];		/*IIS��������������������*/
//unsigned	short	SoundLeftOutData[SOUND_DATA_BUFF_SIZE],SoundRightOutData[SOUND_DATA_BUFF_SIZE];		/*�������������������*/
unsigned	int	SoundTopPoint,SoundBottomPoint;				/*���������վ绺����ͷ��βָ��*/
unsigned	int	SoundDataLength;						/*δ���ŵ���Ӳ���ݳ���*/
unsigned	int	SoundDriveMark;						/*�������ű�־��1�����ţ�0��δ����*/
unsigned int	SoundDrvTop,SoundDrvBottom;				/*������������ͷ��βָ��*/	

//unsigned	short	SoundLeftOutData1[SOUND_DATA_BUFF_SIZE],SoundRightOutData1[SOUND_DATA_BUFF_SIZE];		/*�������������������-1*/
#pragma arm section 
/*==========================================================================================ȫ�ֺ���������====================================================================================*/
/*====================================================��ʼ������ȫ�ֱ���=========================================================*/
void      InitVarent(void)
{
unsigned	int	TempNumber;
      /*========================================����CANͨѶȫ�ֱ�������ֵ=======================================================*/
      Can0TranTop=0;                      /*CAN0���ͻ�����ͷָ��*/
      Can0TranTail=0;                     /*CAN0���ͻ�����βָ��*/
      Can1TranTop=0;                      /*CAN1���ͻ�����ͷָ��*/
      Can1TranTail=0;                     /*CAN1���ͻ�����βָ��*/
      Can2TranTop=0;                      /*CAN2���ͻ�����ͷָ��*/
      Can2TranTail=0;                     /*CAN2���ͻ�����βָ��*/
      Can0RecePosition=0;                 /*Can0���ջ�����ָ��λ��*/
      Can1RecePosition=0;                 /*Can1���ջ�����ָ��λ��*/
      Can2RecePosition=0;                 /*Can2���ջ�����ָ��λ��*/
	
      /*=======================================================================================================================*/
	HostCommandState=0;			/*������������״̬*/
	SystemTicksPrev=0;			/*ϵͳʱ����Ļ���*/
	memset(CcpDataBuff,0,8);		/*CCP�궨���ݻ�����*/
	memset(HostData,0,8);			/*��λ�����͸�����*/
	
	/*======================================================��������=========================================================*/
	memset(IisDataBuffer,0,IIS_RECE_BUFF_SIZE*4);		/*IIS���ݽ��ջ�����*/
	SoundDataLength=0;					/*δ���ŵ���Ӳ���ݳ���*/
	SoundDriveMark=0;						/*�������ű�־��1�����ţ�0��δ����*/
	IisReceTop=0;				/*IIS�������ݻ�������βָ��*/
	IisReceBottom=0;				/*IIS�������ݻ�������βָ��*/
	ReceDataSize=0;				/*����Ϊ�������ݳ���*/
	SoundTopPoint=0;				/*���������վ绺����ͷ��βָ��*/
	SoundBottomPoint=0;			/*���������վ绺����ͷ��βָ��*/
	SoundDrvTop=0;				/*������������ͷ��βָ��*/	
	SoundDrvBottom=0;				/*������������ͷ��βָ��*/	
	for(TempNumber=0;TempNumber<SOUND_DRIVE_BUFF_SIZE;TempNumber++)
	{
		SoundOutputCH1[TempNumber]=833;
		SoundOutputCH2[TempNumber]=833;
		SoundOutputCH3[TempNumber]=833;
		SoundOutputCH4[TempNumber]=833;
	};
	
	DISABLE_FS2_GR1;				/*����ʻ����Ƶ��ѡ��2ʹ�ܣ�PTB0=0*/
	ENABLE_FS1_GR1;				/*����ʻ����Ƶ��ѡ��1ʹ�ܣ�PTB1=1*/	
	DISABLE_FS0_GR1;				/*����ʻ����Ƶ��ѡ��0ʹ�ܣ�PTE2=0*/
	
	DISABLE_FS2_GR4;				/*���ų�Ա2����Ƶ��ѡ��2��ֹ��PTA11=0*/
	ENABLE_FS1_GR4;				/*���ų�Ա2����Ƶ��ѡ��1ʹ�ܣ�PTA12=1*/
	DISABLE_FS0_GR4;				/*���ų�Ա2����Ƶ��ѡ��0��ֹ��PTA13=0*/
	
	DISABLE_FS2_GR3;				/*���ų�Ա1����Ƶ��ѡ��2��ֹ��PTD4=0*/
	ENABLE_FS1_GR3;				/*���ų�Ա1����Ƶ��ѡ��1ʹ�ܣ�PTD3=1*/
  DISABLE_FS0_GR3;				/*���ų�Ա1����Ƶ��ѡ��0��ֹ��PTD2=0*/
		
	FTM0_PWM_init(0,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(1,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(2,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(3,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(4,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(5,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(6,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM0_PWM_init(7,50);			/*��ʼ��FTM0 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(0,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(1,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(2,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(3,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(4,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(5,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(6,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	FTM1_PWM_init(7,50);			/*��ʼ��FTM1 ͨ��0 PWM���ã�ռ�ձ�50%*/
	
	StartFtmCounter(0);    			/*����FTM0������FTM0ģ�鿪ʼ������*/	
	StartFtmCounter(1);    			/*����FTM1������FTM1ģ�鿪ʼ������*/	
	/*=====================================================�����ÿ�ɾ��=======================================================*/
	ENABLE_AD2428_POWER;		      /*ʹ��AD2428��Դ���,PTBE14=1*/
	FlexIOInit();				/*��ʼ��FLEX_IO -->IIS*/
	FlexioDmaRece((unsigned char *)IisDataBuffer,IIS_RECE_BUFF_SIZE);		/*Flex_IO��DMA12���ݴ���*/
	
}
/*======================================================================================================================================================
* ������:             	InputLRData 
* ����:          	 			��I2S�����������������ݷ�����䵽�������뻺����lbufferptr��rbufferptr�У����仺����������δ������Ԫ��
* ����:
*                       ��
* ���:
*   	LeftBufferPtr:        �������뻺������������ַ
*  	LeftBufferSize:       �������뻺������������С
*   	RightBufferPtr:     		�������뻺������������ַ
*   	RightBufferSize: 			�������뻺������������С
* ����ֵ:
*   >0                  ����Ԫ�ظ���
*   0                  	��������ʧ�ܻ��޴�������
=======================================================================================================================================================*/
unsigned int InputLRData(unsigned short *LeftBufferPtr,unsigned int LeftBufferSize,unsigned short *RightBufferPtr,unsigned int RightBufferSize)
{
int TempSize,TempNumber;
	
	IisReceBottom=GetIisDmaPosition(IIS_RECE_BUFF_SIZE);		/*����DMA����IIS�����ڻ������е�λ��:ѭ������βָ��*/
	TempSize=(IisReceBottom+ IIS_RECE_BUFF_SIZE -IisReceTop) % IIS_RECE_BUFF_SIZE;	/*����Ϊ�������ݳ���*/
	ReceDataSize=ReceDataSize+ TempSize;
	/*=====================================��IIS���ݷֱ�д�������������ݻ�������==================================*/
	if((TempSize <= LeftBufferSize)  && (TempSize <= RightBufferSize))	 	/*�������ݳ�����Ч*/
	{
		for(TempNumber=0;TempNumber<TempSize;TempNumber++)
		{
			LeftBufferPtr[TempNumber]=((IisDataBuffer[(IisReceTop + TempNumber) % IIS_RECE_BUFF_SIZE])) & 0XFFFF;
			RightBufferPtr[TempNumber]=(((IisDataBuffer[(IisReceTop + TempNumber) % IIS_RECE_BUFF_SIZE]))>> 16) & 0XFFFF;
		};
	};
	IisReceTop=IisReceBottom;		/*IIS���ջ�����ͷָ�����*/
	return TempSize;
};
/*=====================================================================================================================================================
* ������:             	OutputLRData 
* ����:          	 			��������������������������ݴ��䵽����������deslbufferptr��desrbufferptr�У�����Ԫ�ظ����ɺ�������ָ��
* ����:
*   LeftOutputPtr:     	���������������������ַ
*   LefuOutputSize:    	���������������������Ҫ�����ֽ���
*   RightOutputPtr:     ���������������������ַ
*   RightOutputSize:    ���������������������Ҫ�����ֽ���
* ���:
*   										��
* ����ֵ:
*   >0                  ����Ԫ�ظ���
*   0                  	��������ʧ�ܻ������ݴ���
======================================================================================================================================================*/
unsigned int OutputLRData(unsigned int OutputSize)
{
int	ReturnCode;
unsigned int TempNumber,TempSoundValue;
	ReturnCode=0;
	if(OutputSize <=SOUND_DATA_BUFF_SIZE)	/*��������ݳ�����Ч*/
	{
		SoundDataLength=SoundDataLength + OutputSize;				/*δ���ŵ���Ӳ���ݳ���*/
		/*
		for(TempNumber=0;TempNumber<OutputSize;TempNumber++)	//�������������ݷ��͵����������
		{
			//CH1
			TempSoundValue=((unsigned short)(32768+LeftOutputPtr[TempNumber]))/44+88;
			SoundOutputDriver1[(SoundTopPoint + TempNumber) % SOUND_DRIVE_BUFF_SIZE]=TempSoundValue;
			//CH2
			TempSoundValue=((unsigned short)(32768+RightOutputPtr[TempNumber]))/44+88;
			SoundOutputDriver2[(SoundTopPoint + TempNumber) % SOUND_DRIVE_BUFF_SIZE]=TempSoundValue;
			//CH3
			TempSoundValue=((unsigned short)(32768+LeftOutputPtr1[TempNumber]))/44+88;
			SoundOutputPassenger1[(SoundTopPoint + TempNumber) % SOUND_DRIVE_BUFF_SIZE]=TempSoundValue;
			//CH4
			TempSoundValue=((unsigned short)(32768+RightOutputPtr1[TempNumber]))/44+88;
			SoundOutputPassenger2[(SoundTopPoint + TempNumber) % SOUND_DRIVE_BUFF_SIZE]=TempSoundValue;
		};
		*/
		SoundBottomPoint=(SoundBottomPoint+OutputSize) %SOUND_DRIVE_BUFF_SIZE;		
		if(SoundDriveMark==0)	/*�������ű�־��1�����ţ�0��δ����*/
		{
			if(SoundDataLength > 800)	/*���㹻��δ�������ݿ��Բ���*/
			{
				SoundDriveMark=1;				/*�������ű�־��1�����ţ�0��δ����*/
				ENABLE_EN_GR1;				/*��������ʻ����������ʹ�ܣ�PTD14=1*/
				ENABLE_EN_GR4;				/*���ú��ų�Ա2����������ʹ�ܣ�PTE6=1*/
				ENABLE_EN_GR3;				/*���ú��ų�Ա1����������ʹ�ܣ�PTA14=1*/
				Fmt0Dma0Tran((unsigned char *)(&SoundOutputCH1[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT0��DMA0���ݴ���*/
				Fmt1Dma1Tran((unsigned char *)(&SoundOutputCH2[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT1��DMA1���ݴ���*/
				Fmt1Dma2Tran((unsigned char *)(&SoundOutputCH3[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT1��DMA2���ݴ���*/
				Fmt1Dma3Tran((unsigned char *)(&SoundOutputCH4[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT1��DMA3���ݴ���*/
			};
		}else
		{
			SoundDrvBottom=GetSoundDrvPosition(SOUND_DRIVE_BUFF_SIZE);	/*����DMA�������������ڻ������е�λ��*/	
			TempNumber=(SoundDrvBottom + SOUND_DRIVE_BUFF_SIZE -SoundDrvTop) % SOUND_DRIVE_BUFF_SIZE;
			SoundDataLength=SoundDataLength - TempNumber;
			if(SoundDataLength < 768)		/*�������ݲ�������*/
			{
				SoundDriveMark=0;				/*�������ű�־��1�����ţ�0��δ����*/
				DISABLE_EN_GR1;				/*��ֹ����ʻ����������ʹ�ܣ�PTD14=0*/
				DISABLE_EN_GR4;				/*��ֹ���ų�Ա2����������ʹ�ܣ�PTE6=0*/
				DISABLE_EN_GR3;				/*��ֹ���ų�Ա1����������ʹ�ܣ�PTA14*/
				DisableDma(0);				/*�ر���Ӧ��DMA0*/
				DisableDma(1);				/*�ر���Ӧ��DMA1*/
				DisableDma(2);				/*�ر���Ӧ��DMA2*/
				DisableDma(3);				/*�ر���Ӧ��DMA3*/
			};
			SoundDrvTop=SoundDrvBottom;
		};
		SoundTopPoint=SoundBottomPoint;
		ReturnCode=OutputSize;
	};
	return ReturnCode;
};





