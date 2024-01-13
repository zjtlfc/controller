#include "S32K144_API.h"
#include "system_S32K144.h"             // Keil::Device:Startup
#include "clock_S32K1xx.h"              // Keil::Device:SDK Driver:Clock
#include "clock_manager.h"              // Keil::Device:SDK Driver:Clock
#include "string.h"
//#include "S32K144.h"
#include "function.h"
#include "SystemConfig.h"
#include "s32_core_cm4.h"
/*====================================================================宏定义区==============================================================*/
/*===================================================================常量定义区=============================================================*/
/*=================================================================全局变量定义区===========================================================*/
struct      CanRepeatTran     CanRepeatBuff[CAN_REPEAT_BUFFER_SIZE]=                /*CAN循环发送缓冲区定义*/
      {     /*   周期  当前时间     CAN_ID           数据长度 动作 ID类型  通道     发送数量     数据值    */
                  10,   10,   BMU_TO_BCU_CONTROL,          8, 0XAA,  0,  SYSTEM_CAN, -1,       {0X0C,1,0,0,0,0,0,0},                 /*读BCU所有数据 电芯电压、温度ID:0X20*/
                  10,   10,   BMS_RUN_MESSAGE,             8, 0X55,  1,  SYSTEM_CAN, -1,       {0,0,0,0,0,0,0,0},                    /*BMS系统运行状态信息,ID=0X012D11EB(19730923)*/
                  100,  100,  CELL_STATUS_MESSAGE,         8, 0X55,  1,  SYSTEM_CAN, 0,       {0,0,0,0,0,0,0,0},                    /*电池包电芯状态信息,ID=0X012DFBE8(19790824)*/
                  500, 500,   BMS_AUX1_MESSAGE,            8, 0X55,  1,  SYSTEM_CAN, 0,       {0,0,0,0,0,0,0,0},                    /*BMS运行辅助信息,ID=0X013218FA(20060410)*/
      };          
unsigned 	int	SysTickTime;		/*系统检测节拍时间,精度10mS*/  
unsigned 	int	SystemTimes;    		/*获取系统上电后的运行时间,单位S*/
unsigned   int	IisDataBuffer[IIS_RECE_BUFF_SIZE+5];				/*IIS数据接收缓冲区*/
unsigned	int	IisReceTop,IisReceBottom;					/*IIS接收数据缓冲区首尾指针*/
unsigned	int	ReceDataSize;							/*接收为处理数据长度*/
			
unsigned	short		SoundOutputCH1[SOUND_DRIVE_BUFF_SIZE+5];	/*声音输出1（数据：0—1666）*/
unsigned	short		SoundOutputCH2[SOUND_DRIVE_BUFF_SIZE+5];	/*声音输出2（数据：0—1666）*/	
unsigned	short		SoundOutputCH3[SOUND_DRIVE_BUFF_SIZE+5];	/*声音输出3（数据：0—1666）*/
unsigned	short		SoundOutputCH4[SOUND_DRIVE_BUFF_SIZE+5];	/*声音输出4（数据：0—1666）*/	
			
unsigned	short	IisLeftSoundData[SOUND_DATA_BUFF_SIZE],IisRightSoundData[SOUND_DATA_BUFF_SIZE];		/*IIS左右声道输入声音数据*/
//unsigned	short	SoundLeftOutData[SOUND_DATA_BUFF_SIZE],SoundRightOutData[SOUND_DATA_BUFF_SIZE];		/*声音输出左右声道数据*/
unsigned	int	SoundTopPoint,SoundBottomPoint;				/*左右声道苏剧缓冲区头、尾指针*/
unsigned	int	SoundDataLength;						/*未播放的生硬数据长度*/
unsigned	int	SoundDriveMark;						/*声音播放标志，1：播放；0：未播放*/
unsigned int	SoundDrvTop,SoundDrvBottom;				/*声音波峰数据头、尾指针*/	

//unsigned	short	SoundLeftOutData1[SOUND_DATA_BUFF_SIZE],SoundRightOutData1[SOUND_DATA_BUFF_SIZE];		/*声音输出左右声道数据-1*/
#pragma arm section 
/*==========================================================================================全局函数定义区====================================================================================*/
/*====================================================初始化所有全局变量=========================================================*/
void      InitVarent(void)
{
unsigned	int	TempNumber;
      /*========================================关于CAN通讯全局变量赋初值=======================================================*/
      Can0TranTop=0;                      /*CAN0发送缓冲区头指针*/
      Can0TranTail=0;                     /*CAN0发送缓冲区尾指针*/
      Can1TranTop=0;                      /*CAN1发送缓冲区头指针*/
      Can1TranTail=0;                     /*CAN1发送缓冲区尾指针*/
      Can2TranTop=0;                      /*CAN2发送缓冲区头指针*/
      Can2TranTail=0;                     /*CAN2发送缓冲区尾指针*/
      Can0RecePosition=0;                 /*Can0接收缓冲区指针位置*/
      Can1RecePosition=0;                 /*Can1接收缓冲区指针位置*/
      Can2RecePosition=0;                 /*Can2接收缓冲区指针位置*/
	
      /*=======================================================================================================================*/
	HostCommandState=0;			/*主机发送命令状态*/
	SystemTicksPrev=0;			/*系统时间节拍缓冲*/
	memset(CcpDataBuff,0,8);		/*CCP标定数据缓冲区*/
	memset(HostData,0,8);			/*上位机发送给数据*/
	
	/*======================================================声音驱动=========================================================*/
	memset(IisDataBuffer,0,IIS_RECE_BUFF_SIZE*4);		/*IIS数据接收缓冲区*/
	SoundDataLength=0;					/*未播放的生硬数据长度*/
	SoundDriveMark=0;						/*声音播放标志，1：播放；0：未播放*/
	IisReceTop=0;				/*IIS接收数据缓冲区首尾指针*/
	IisReceBottom=0;				/*IIS接收数据缓冲区首尾指针*/
	ReceDataSize=0;				/*接收为处理数据长度*/
	SoundTopPoint=0;				/*左右声道苏剧缓冲区头、尾指针*/
	SoundBottomPoint=0;			/*左右声道苏剧缓冲区头、尾指针*/
	SoundDrvTop=0;				/*声音波峰数据头、尾指针*/	
	SoundDrvBottom=0;				/*声音波峰数据头、尾指针*/	
	for(TempNumber=0;TempNumber<SOUND_DRIVE_BUFF_SIZE;TempNumber++)
	{
		SoundOutputCH1[TempNumber]=833;
		SoundOutputCH2[TempNumber]=833;
		SoundOutputCH3[TempNumber]=833;
		SoundOutputCH4[TempNumber]=833;
	};
	
	DISABLE_FS2_GR1;				/*主驾驶驱动频率选择2使能，PTB0=0*/
	ENABLE_FS1_GR1;				/*主驾驶驱动频率选择1使能，PTB1=1*/	
	DISABLE_FS0_GR1;				/*主驾驶驱动频率选择0使能，PTE2=0*/
	
	DISABLE_FS2_GR4;				/*后排成员2驱动频率选择2禁止，PTA11=0*/
	ENABLE_FS1_GR4;				/*后排成员2驱动频率选择1使能，PTA12=1*/
	DISABLE_FS0_GR4;				/*后排成员2驱动频率选择0禁止，PTA13=0*/
	
	DISABLE_FS2_GR3;				/*后排成员1驱动频率选择2禁止，PTD4=0*/
	ENABLE_FS1_GR3;				/*后排成员1驱动频率选择1使能，PTD3=1*/
  DISABLE_FS0_GR3;				/*后排成员1驱动频率选择0禁止，PTD2=0*/
		
	FTM0_PWM_init(0,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(1,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(2,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(3,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(4,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(5,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(6,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM0_PWM_init(7,50);			/*初始化FTM0 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(0,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(1,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(2,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(3,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(4,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(5,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(6,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	FTM1_PWM_init(7,50);			/*初始化FTM1 通道0 PWM配置，占空比50%*/
	
	StartFtmCounter(0);    			/*开启FTM0计数：FTM0模块开始工作。*/	
	StartFtmCounter(1);    			/*开启FTM1计数：FTM1模块开始工作。*/	
	/*=====================================================调试用可删除=======================================================*/
	ENABLE_AD2428_POWER;		      /*使能AD2428电源输出,PTBE14=1*/
	FlexIOInit();				/*初始化FLEX_IO -->IIS*/
	FlexioDmaRece((unsigned char *)IisDataBuffer,IIS_RECE_BUFF_SIZE);		/*Flex_IO，DMA12数据传输*/
	
}
/*======================================================================================================================================================
* 函数名:             	InputLRData 
* 描述:          	 			将I2S缓冲区左右声道数据分离后传输到运算输入缓冲区lbufferptr和rbufferptr中，传输缓冲区中所有未传出的元素
* 输入:
*                       无
* 输出:
*   	LeftBufferPtr:        运算输入缓冲区左声道地址
*  	LeftBufferSize:       运算输入缓冲区左声道大小
*   	RightBufferPtr:     		运算输入缓冲区右声道地址
*   	RightBufferSize: 			运算输入缓冲区右声道大小
* 返回值:
*   >0                  传输元素个数
*   0                  	函数调用失败或无传输数据
=======================================================================================================================================================*/
unsigned int InputLRData(unsigned short *LeftBufferPtr,unsigned int LeftBufferSize,unsigned short *RightBufferPtr,unsigned int RightBufferSize)
{
int TempSize,TempNumber;
	
	IisReceBottom=GetIisDmaPosition(IIS_RECE_BUFF_SIZE);		/*返回DMA接收IIS数据在缓冲区中的位置:循环队列尾指针*/
	TempSize=(IisReceBottom+ IIS_RECE_BUFF_SIZE -IisReceTop) % IIS_RECE_BUFF_SIZE;	/*接收为处理数据长度*/
	ReceDataSize=ReceDataSize+ TempSize;
	/*=====================================将IIS数据分别写入左右声道数据缓冲区中==================================*/
	if((TempSize <= LeftBufferSize)  && (TempSize <= RightBufferSize))	 	/*传输数据长度有效*/
	{
		for(TempNumber=0;TempNumber<TempSize;TempNumber++)
		{
			LeftBufferPtr[TempNumber]=((IisDataBuffer[(IisReceTop + TempNumber) % IIS_RECE_BUFF_SIZE])) & 0XFFFF;
			RightBufferPtr[TempNumber]=(((IisDataBuffer[(IisReceTop + TempNumber) % IIS_RECE_BUFF_SIZE]))>> 16) & 0XFFFF;
		};
	};
	IisReceTop=IisReceBottom;		/*IIS接收缓冲区头指针调整*/
	return TempSize;
};
/*=====================================================================================================================================================
* 函数名:             	OutputLRData 
* 描述:          	 			将运算输出缓冲区左右声道数据传输到驱动缓冲区deslbufferptr和desrbufferptr中，传输元素个数由函数参数指定
* 输入:
*   LeftOutputPtr:     	运算输出缓冲区左声道地址
*   LefuOutputSize:    	运算输出缓冲区左声道需要传输字节数
*   RightOutputPtr:     运算输出缓冲区右声道地址
*   RightOutputSize:    运算输出缓冲区右声道需要传输字节数
* 输出:
*   										无
* 返回值:
*   >0                  传输元素个数
*   0                  	函数调用失败或无数据传输
======================================================================================================================================================*/
unsigned int OutputLRData(unsigned int OutputSize)
{
int	ReturnCode;
unsigned int TempNumber,TempSoundValue;
	ReturnCode=0;
	if(OutputSize <=SOUND_DATA_BUFF_SIZE)	/*传输的数据长度有效*/
	{
		SoundDataLength=SoundDataLength + OutputSize;				/*未播放的生硬数据长度*/
		/*
		for(TempNumber=0;TempNumber<OutputSize;TempNumber++)	//将左右声道数据发送的输出缓冲区
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
		if(SoundDriveMark==0)	/*声音播放标志，1：播放；0：未播放*/
		{
			if(SoundDataLength > 800)	/*有足够的未播放数据可以播放*/
			{
				SoundDriveMark=1;				/*声音播放标志，1：播放；0：未播放*/
				ENABLE_EN_GR1;				/*设置主驾驶座椅驱动器使能，PTD14=1*/
				ENABLE_EN_GR4;				/*设置后排成员2座椅驱动器使能，PTE6=1*/
				ENABLE_EN_GR3;				/*设置后排成员1座椅驱动器使能，PTA14=1*/
				Fmt0Dma0Tran((unsigned char *)(&SoundOutputCH1[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT0，DMA0数据传输*/
				Fmt1Dma1Tran((unsigned char *)(&SoundOutputCH2[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT1，DMA1数据传输*/
				Fmt1Dma2Tran((unsigned char *)(&SoundOutputCH3[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT1，DMA2数据传输*/
				Fmt1Dma3Tran((unsigned char *)(&SoundOutputCH4[0]),SOUND_DRIVE_BUFF_SIZE);		/*FMT1，DMA3数据传输*/
			};
		}else
		{
			SoundDrvBottom=GetSoundDrvPosition(SOUND_DRIVE_BUFF_SIZE);	/*返回DMA声音驱动数据在缓冲区中的位置*/	
			TempNumber=(SoundDrvBottom + SOUND_DRIVE_BUFF_SIZE -SoundDrvTop) % SOUND_DRIVE_BUFF_SIZE;
			SoundDataLength=SoundDataLength - TempNumber;
			if(SoundDataLength < 768)		/*声音数据不够播放*/
			{
				SoundDriveMark=0;				/*声音播放标志，1：播放；0：未播放*/
				DISABLE_EN_GR1;				/*禁止主驾驶座椅驱动器使能，PTD14=0*/
				DISABLE_EN_GR4;				/*禁止后排成员2座椅驱动器使能，PTE6=0*/
				DISABLE_EN_GR3;				/*禁止后排成员1座椅驱动器使能，PTA14*/
				DisableDma(0);				/*关闭相应的DMA0*/
				DisableDma(1);				/*关闭相应的DMA1*/
				DisableDma(2);				/*关闭相应的DMA2*/
				DisableDma(3);				/*关闭相应的DMA3*/
			};
			SoundDrvTop=SoundDrvBottom;
		};
		SoundTopPoint=SoundBottomPoint;
		ReturnCode=OutputSize;
	};
	return ReturnCode;
};





