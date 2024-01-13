

/*===============================================================宏定义区========================================================*/
/*=====================================与声音驱动相关宏定义=======================================================*/
#define	IIS_RECE_BUFF_SIZE		1000			/*IIS数据接收缓冲区大小*/
#define	SOUND_DRIVE_BUFF_SIZE		2000			/*声音驱动数据缓冲区大小*/
#define	SOUND_DATA_BUFF_SIZE		1000			/*生硬输入、输出缓冲区大小（16mS周期）*/
/*=====================================BMS CAN总线报文ID==========================================================*/
#define     BMU_TO_BCU_CONTROL            0X20              /*BMU发送命令数据给BCU，控制BCU运行,ID:0X20*/
#define     BMS_RUN_MESSAGE               0X012D11EB        /*BMS系统运行状态信息,ID=0X012D11EB(19730923)*/
#define     CELL_STATUS_MESSAGE           0X012DFBE8        /*电池包电芯状态信息,ID=0X012DFBE8(19790824)*/
#define     BMS_AUX1_MESSAGE              0X013218FA        /*BMS运行辅助信息1,ID=0X013218FA(20060410)*/
#define     BMS_AUX2_MESSAGE              0X01282D2E        /*BMS运行辅助信息2,ID=0X01282D2E(19410222)*/
#define     BMS_ERROR_MESSAGE             0X012EE511        /*BMS在内CAN中上报报警信息,ID=0X012EE511(19410222)*/
#define     BMS_CHARGE_MESSAGE            0X0129DAD7        /*BMS发出与充电信息相关数据,ID=0X0129DAD7(19520215)*/
#define     BMS_BALANCE_MESSAGE           0X012E4AC3        /*BMS系统动态平衡相关信息,ID=0X012E4AC3(19811011)*/
#define     BMU_TO_SLOW_CHANGR            0X1806E5F4        /*BMU发送给充电机的参数数据,ID:0X1806E5F4*/
#define     BMS_INTRE_TEST_MESSAGE        0X18FF0000        /*BMS内部测试用CAN报文,数据格式不定,ID:0X18FF0000*/
/*======================================与CAN发送相关宏定义===================================*/
#define     CAN_REPEAT_BUFFER_SIZE        4    /*CAN循环发送数据缓冲区数量*/
//#define     CHARGE_CAN                    CanChannel2       /*充电CAN为CAN通道2*/
//#define     BATTERY_CAN                   CanChannel1       /*BMS内部CAN为CAN通道1*/
#define     SYSTEM_CAN                    CanChannel0       /*整车CAN为CAN通道0*/
/*==================================重复发送CAN数据结构======================================*/
struct  CanRepeatTran                             
{
unsigned int      RepeatTimes;                  /*CAN数据发送循环间隔时间,单位mS*/
unsigned int      SurplusTimes;                 /*当前CAN数据发送的剩余时间,单位mSmS*/
unsigned int      CanID;                        /*当前要发送CAN数据的ID信息*/
unsigned char     DataLength;                   /*当前要发送CAN数据长度*/
unsigned char     Action;                       /*要在CAN通讯过程中执行的动作0:无动作   0X55:CAN发送前执行数据更新  0XAA:CAN发送后执行数据更新*/
unsigned char     CanIdType;                    /*CAN报文类型；0：标准帧；1：扩展帧*/
unsigned char     CanChannel;                   /*CAN通道号*/
int               Times;                        /*发送的次数 >0:发送若干次, =0:停止发送 <0:永远循环发送*/
unsigned char     Data[8];                      /*当前要发送CAN数据*/
};
/*==============================================================变量声明区============================================================*/
extern	const	unsigned short  SoundTest1[];							/*声音测试数据，48KHz*/
extern	struct      CanRepeatTran     CanRepeatBuff[];					/*CAN数据发送缓冲区*/
extern	unsigned int	SysTickTime;							/*系统检测节拍时间,精度10mS*/  
extern	unsigned int	SystemTimes;    							/*获取系统上电后的运行时间,单位S*/
extern	unsigned   int	IisDataBuffer[];							/*IIS数据接收缓冲区*/
extern	unsigned	int	IisReceTop,IisReceBottom;					/*IIS接收数据缓冲区首尾指针*/
extern	unsigned	int	ReceDataSize;							/*接收为处理数据长度*/
extern	unsigned	short		SoundOutputCH1[];					/*驾驶员座椅声音输出1（数据：0—1666）*/
extern	unsigned	short		SoundOutputCH2[];					/*驾驶员座椅声音输出2（数据：0—1666）*/	
extern	unsigned	int	SoundTopPoint,SoundBottomPoint;				/*左右声道苏剧缓冲区头、尾指针*/
extern	unsigned	short	IisLeftSoundData[],IisRightSoundData[];			/*IIS左右声道输入声音数据*/
//extern	unsigned	short	SoundLeftOutData[],SoundRightOutData[];			/*声音输出左右声道数据*/
extern	unsigned	int	SoundDataLength;							/*未播放的生硬数据长度*/
extern	unsigned	int	SoundDriveMark;							/*声音播放标志，1：播放；0：未播放*/
extern	unsigned int	SoundDrvTop,SoundDrvBottom;					/*声音波峰数据头、尾指针*/	
extern	unsigned	short		SoundOutputCH3[];				/*前乘员座椅声音输出1（数据：0—1666）*/
extern	unsigned	short		SoundOutputCH4[];				/*前乘员座椅声音输出2（数据：0—1666）*/	
//extern	unsigned	short	SoundLeftOutData1[],SoundRightOutData1[];			/*声音输出左右声道数据-1*/

/*====================================================全局函数声明区============================================*/
extern      void              InitVarent(void);                                     /*初始化所有全局变量*/
extern   unsigned   int InputLRData(unsigned short *,unsigned int,unsigned short *,unsigned int);		/*将系统通过IIS接受的数据发送到相应的左右声道缓冲区中*/
extern   unsigned   int OutputLRData(unsigned int);	/*将运算输出缓冲区左右声道数据传输到驱动缓冲区*/




