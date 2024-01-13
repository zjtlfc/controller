
#define	OSC		8							/*定义晶振为8MHZ*/
#define     WATCHDOG_RESET_SYSTEM   WDOG->TOVAL = (uint32_t )0;               /*由看门狗触发的系统复位*/
#define	SOFTWARE_RESET	S32_SCB->AIRCR = S32_SCB_AIRCR_VECTKEY(0x5FA) | S32_SCB_AIRCR_SYSRESETREQ(1)	/*由软件引起得复位*/
#define	MASTER_COMMAND_CANAPE_CALIBRATIOB	0X00FF0000				/*系统支持CANAPE标定功能*/
#define	MASTER_COMMAND_PARAMETER_TRANS	0X000000FF				/*系统支持参数下载功能*/
#define	MASTER_COMMAND_UDS_DIAGONSE		0XFF000000				/*系统支持UDS诊断功能*/
#define	MASTER_COMMAND_VARENT_UPLOAD		0X0000FF00				/*系统支持变量数据上传功能*/
#define	RESET_VECTOR_HANDLE 			1					/*手动强制定位复位向量地址，1：使能；0：禁止*/
#define	ATD_FILTER_SCALE				8					/*A/D转换滤波比例*/
/*===========================================定义所有输入宏定义===========================================*/
#define	KL15_WAKE_SW		((PTD->PDIR & 0X00000080)>>7)      		/*KL_15钥匙唤醒输入信号PTD7*/	
#define	CAN_WAKE_SW			((PTC->PDIR & 0X00000100)>>8)      		/*CAN唤醒输入信号PTC8*/	
#define	DRIVER_DIAGON11		((PTB->PDIR & 0X00004000)>>14)		/*驱动模块11诊断输入信号PTB14*/
#define	DRIVER_DIAGON12		((PTB->PDIR & 0X00008000)>>15)		/*驱动模块12诊断输入信号PTB15*/
#define	DRIVER_DIAGON21		((PTB->PDIR & 0X00002000)>>13)		/*驱动模块21诊断输入信号PTB13*/
#define	DRIVER_DIAGON22		((PTB->PDIR & 0X00001000)>>12)		/*驱动模块22诊断输入信号PTB12*/
#define	DRIVER_DIAGON31		((PTB->PDIR & 0X00020000)>>17)		/*驱动模块31诊断输入信号PTB17*/
#define	DRIVER_DIAGON32		((PTB->PDIR & 0X00010000)>>16)		/*驱动模块32诊断输入信号PTB16*/
#define	DRIVER_DIAGON41		((PTB->PDIR & 0X00000800)>>11)		/*驱动模块41诊断输入信号PTB11*/
#define	DRIVER_DIAGON42		((PTB->PDIR & 0X00000100)>>8)			/*驱动模块42诊断输入信号PTB8*/
/*==========================================定义所有输出宏定义============================================*/
#define	WATCHDOG_TRIGGER		PTC->PTOR = 0X00000200		            /*触发看门狗喂食,PTC9翻转*/ 
/*========================================================*/
#define	POWER_LOCK_ENABLE		PTC->PSOR = 0X00000400		            /*设置使能电源自锁,PTC10=1*/
#define	ENABLE_AD2428_POWER	PTE->PSOR = 0X00004000		            /*使能AD2428电源输出,PTBE14=1*/

#define	DISABLE_PTD_PD0		PTD->PCOR = 0X00000001				/*，PTD0=0*/
#define	DISABLE_PTD_PD1		PTD->PCOR = 0X00000002				/*，PTD1=0*/
#define	DISABLE_SCL_CLEAR		PTA->PCOR = 0X00000008				/*，PTA3=0*/
#define	DISABLE_SDA_CLEAR		PTA->PCOR = 0X00000004				/*，PTA2=0*/
/*========================================================*/
#define	POWER_LOCK_DISABLE	PTC->PCOR = 0X00000400		            /*设置关闭电源自锁,PTC10=0*/
#define	DISABLE_AD2428_POWER	PTE->PCOR = 0X00004000		            /*禁止AD2428电源输出,PTE14=0*/
/*========================================================*/
#define	CAN_EN_ENABLE		PTE->PSOR = 0X00000002				/*使能CAN_EN,PTE1=1*/
#define	CAN_STB_ENABLE		PTE->PSOR = 0X00000001				/*使能CAN_STB,PTE0=1*/	
#define	CAN_EN_DISABLE		PTE->PCOR = 0X00000002				/*使能CAN_EN,PTE1=0*/
#define	CAN_STB_DISABLE		PTE->PCOR = 0X00000001				/*使能CAN_STB,PTE0=0*/	
/*=======================主驾驶座椅驱动器设置=====================*/
#define	ENABLE_STANDBY_GR1	PTD->PSOR = 0X00002000				/*设置主驾驶座椅驱动器STANBY模式，PTD13=1*/
#define	ENABLE_EN_GR1		PTD->PSOR = 0X00004000				/*设置主驾驶座椅驱动器使能，PTD14=1*/
#define	ENABLE_FS0_GR1		PTE->PSOR = 0X00000004				/*主驾驶驱动频率选择0使能，PTE2=1*/
#define	ENABLE_FS1_GR1		PTB->PSOR = 0X00000002				/*主驾驶驱动频率选择1使能，PTB1=1*/
#define	ENABLE_FS2_GR1		PTB->PSOR = 0X00000001				/*主驾驶驱动频率选择2使能，PTB0=1*/
#define	DISABLE_STANDBY_GR1	PTD->PCOR = 0X00002000				/*清除主驾驶座椅驱动器STANBY模式，PTD13=0*/
#define	DISABLE_EN_GR1		PTD->PCOR = 0X00004000				/*禁止主驾驶座椅驱动器使能，PTD14=0*/
#define	DISABLE_FS0_GR1		PTE->PCOR = 0X00000004				/*主驾驶驱动频率选择0禁止，PTE2=0*/
#define	DISABLE_FS1_GR1		PTB->PCOR = 0X00000002				/*主驾驶驱动频率选择1禁止，PTB1=0*/
#define	DISABLE_FS2_GR1		PTB->PCOR = 0X00000001				/*主驾驶驱动频率选择2禁止，PTB0=0*/
/*=======================前排成员座椅驱动器设置=====================*/
#define	ENABLE_STANDBY_GR2	PTD->PSOR = 0X00000020				/*设置前排成员座椅驱动器STANBY模式，PTD5=1*/
#define	ENABLE_EN_GR2		PTD->PSOR = 0X00000040				/*设置前排成员座椅驱动器使能，PTD6=1*/
#define	ENABLE_FS0_GR2		PTD->PSOR = 0X00000400				/*前排成员驱动频率选择0使能，PTD10=1*/
#define	ENABLE_FS1_GR2		PTD->PSOR = 0X00000800				/*前排成员驱动频率选择1使能，PTD11=1*/
#define	ENABLE_FS2_GR2		PTD->PSOR = 0X00001000				/*前排成员驱动频率选择2使能，PTD12=1*/
#define	DISABLE_STANDBY_GR2	PTD->PCOR = 0X00000020				/*清除前排成员座椅驱动器STANBY模式，PTD5=0*/
#define	DISABLE_EN_GR2		PTD->PCOR = 0X00000040				/*禁止前排成员座椅驱动器使能，PTD6=0*/
#define	DISABLE_FS0_GR2		PTD->PCOR = 0X00000400				/*前排成员驱动频率选择0禁止，PTD10=0*/
#define	DISABLE_FS1_GR2		PTD->PCOR = 0X00000800				/*前排成员驱动频率选择1禁止，PTD11=0*/
#define	DISABLE_FS2_GR2		PTD->PCOR = 0X00001000				/*前排成员驱动频率选择2禁止，PTD12=0*/
/*=======================后排成员1座椅驱动器设置=====================*/
#define	ENABLE_STANDBY_GR3	PTA->PSOR = 0X00020000				/*设置后排成员1座椅驱动器STANBY模式，PTA17=1*/
#define	ENABLE_EN_GR3		PTA->PSOR = 0X00004000				/*设置后排成员1座椅驱动器使能，PTA14=1*/
#define	ENABLE_FS0_GR3		PTD->PSOR = 0X00000004				/*后排成员1驱动频率选择0使能，PTD2=1*/
#define	ENABLE_FS1_GR3		PTD->PSOR = 0X00000008				/*后排成员1驱动频率选择1使能，PTD3=1*/
#define	ENABLE_FS2_GR3		PTD->PSOR = 0X00000010				/*后排成员1驱动频率选择2使能，PTD4=1*/
#define	DISABLE_STANDBY_GR3	PTA->PCOR = 0X00020000				/*清除后排成员1座椅驱动器STANBY模式，PTA17=0*/
#define	DISABLE_EN_GR3		PTA->PCOR = 0X00004000				/*禁止后排成员1座椅驱动器使能，PTA14*/
#define	DISABLE_FS0_GR3		PTD->PCOR = 0X00000004				/*后排成员1驱动频率选择0禁止，PTD2=0*/
#define	DISABLE_FS1_GR3		PTD->PCOR = 0X00000008				/*后排成员1驱动频率选择1禁止，PTD3=0*/
#define	DISABLE_FS2_GR3		PTD->PCOR = 0X00000010				/*后排成员1驱动频率选择2禁止，PTD4=0*/
/*=======================后排成员2座椅驱动器设置=====================*/
#define	ENABLE_STANDBY_GR4	PTA->PSOR = 0X00008000				/*设置后排成员2座椅驱动器STANBY模式，PTA15=1*/
#define	ENABLE_EN_GR4		PTE->PSOR = 0X00000040				/*设置后排成员2座椅驱动器使能，PTE6=1*/
#define	ENABLE_FS0_GR4		PTA->PSOR = 0X00002000				/*后排成员2驱动频率选择0使能，PTA13=1*/
#define	ENABLE_FS1_GR4		PTA->PSOR = 0X00001000				/*后排成员2驱动频率选择1使能，PTA12=1*/
#define	ENABLE_FS2_GR4		PTA->PSOR = 0X00000800				/*后排成员2驱动频率选择2使能，PTA11=1*/
#define	DISABLE_STANDBY_GR4	PTA->PCOR = 0X00008000				/*清除后排成员2座椅驱动器STANBY模式，PTA15=0*/
#define	DISABLE_EN_GR4		PTE->PCOR = 0X00000040				/*禁止后排成员2座椅驱动器使能，PTE6*/
#define	DISABLE_FS0_GR4		PTA->PCOR = 0X00002000				/*后排成员2驱动频率选择0禁止，PTA13=0*/
#define	DISABLE_FS1_GR4		PTA->PCOR = 0X00001000				/*后排成员2驱动频率选择1禁止，PTA12=0*/
#define	DISABLE_FS2_GR4		PTA->PCOR = 0X00000800				/*后排成员2驱动频率选择2禁止，PTA11=0*/
/*================================================================*/
/*==================================所有输出端口的驱动状态检测=========================================*/
/*=============================================ATD0转换通道定义=============================================*/
#define     UB_VOLTAGE_CHANNEL         	0                             /*系统供电UB电压A/D转换通道*/
#define     AD2428_VOUT2_CHANNEL          1                             /*AD2424输出电压Vout电压A/D转换通道*/
#define     AD2428_POWER_CHANNEL       	2                             /*AD 2428供电电压A/D转换通道*/
//#define     SLOW_CHARGE_CUR_CHANNEL       3                             /*慢充接触器线圈驱动电流A/D转换通道*/
//#define     QUICK_CHARGE_CUR_CHANNEL      4                             /*快充接触器线圈驱动电流A/D转换通道*/
//#define     AC_RELAY_CUR_CHANNEL          5                             /*空调接触器线圈驱动电流A/D转换通道*/
/*=============================================ATD1转换通道定义=============================================*/
/*===============================================EEPROM相关宏定义===========================================*/
#define     ERASR_FLASH_SECTOR      0x09                                /*擦除FLASH扇区中的所有数据*/
#define     PROGRAM_FLASH           0x07                                /*对FLASH进行一次写入程序*/
#define     EEPROM_DEPART           8                                   /*EEPROM分区代码,64KEFLASH*/
#define     EEPROM_EEESIZE          0X02                                /*EEPROM大小:4KByte*/
#define     EEPROM_PGMPART          0X80                                /*对EEPROM进行分区*/
#define     EEPROM_SE_FlEXRAM       0X81                                /*设置FLEXRAM工作模式*/
#define     EEPROM_EMULATED         0X00                                /*FLEXRAM工作在仿真EEPROM模式*/
#define     EEPROM_QUERY_STATUS     0X77                                /*FLEXRAM工作状态查询*/
#define     EEPROM_QUICK_WRITE      0X55                                /*FLEXRAM快速写入操作*/
/*=================================================数据结构定义=============================================*/
/*=================================================FLEXTIMERdingyi==========================================*/
#define	FMT0_DMA_SOURCE		36						/*FTM0模块DMA源*/
#define	FTM1_CH0_DMA_SROUCE	20						/*FTM1模块通道0 DMA源*/
#define	FTM1_CH1_DMA_SROUCE	21						/*FTM1模块通道1 DMA源*/
#define	FTM1_CH2_DMA_SROUCE	22						/*FTM1模块通道2 DMA源*/
#define	FTM1_CH3_DMA_SROUCE	23						/*FTM1模块通道3 DMA源*/
#define	FTM1_CH4_DMA_SROUCE	24						/*FTM1模块通道4 DMA源*/
#define	FTM1_CH5_DMA_SROUCE	25						/*FTM1模块通道5 DMA源*/
#define	FTM1_CH6_DMA_SROUCE	26						/*FTM1模块通道6 DMA源*/
#define	FTM1_CH7_DMA_SROUCE	27						/*FTM1模块通道7 DMA源*/
/*=================================================CAN波特率定义============================================*/
typedef	enum {CAN_1000 ,CAN_500 ,CAN_250 , CAN_200 , CAN_125 , CAN_100 , CAN_50 } CANBAUD;
/*===================================================定义CAN口号============================================*/
typedef	enum { CanChannel0 ,CanChannel1 ,CanChannel2 } CANCHANNEL;
/*===================================================定义SPI通道号==========================================*/
typedef	enum { SpiChannel0 ,SpiChannel1 ,SpiChannel2 } SPICHANNEL;
/*==========================================================================================================*/
#define	CAN0_RECE_MN_SIZE		31					      /*CAN0接收数据邮箱长度*/
#define	CAN0_TRAN_MN_POS		31					      /*CAN0发送数据邮箱位置*/
#define	CAN1_RECE_MN_SIZE		5					      /*CAN1接收数据邮箱长度*/
#define	CAN1_TRAN_MN_POS		5					      /*CAN1发送数据邮箱位置*/
#define	CAN2_RECE_MN_SIZE		5					      /*CAN2接收数据邮箱长度*/
#define	CAN2_TRAN_MN_POS		5					      /*CAN2发送数据邮箱位置*/
#define     CAN0_TRAN_BUFF_SIZE     40                                  /*CAN0发送缓冲区长度*/
#define     CAN1_TRAN_BUFF_SIZE     5                                  /*CAN1发送缓冲区长度*/
#define     CAN2_TRAN_BUFF_SIZE     5                                  /*CAN2发送缓冲区长度*/
#define     CAN0_RECE_BUFF_SIZE     32                                  /*CAN0接受缓冲区长度*/
#define     CAN1_RECE_BUFF_SIZE     5                                  /*CAN1接受缓冲区长度*/
#define     CAN2_RECE_BUFF_SIZE     5                                  /*CAN2接受缓冲区长度*/
#define     DMA_CAN0_SOURCE         54                                  /*CAN0接收DMA源*/
#define     DMA_CAN1_SOURCE         55                                  /*CAN1接收DMA源*/
#define     DMA_CAN2_SOURCE         56                                  /*CAN2接收DMA源*/

#define	DMA_FLEXIO_SHIFTER0_SOURCE	10					/*FLEX_IO移位寄存器0，DMA源*/
/*=========================================CAN消息邮箱RX命令宏定义=========================================*/
#define	CAN_MB_RX_INACTIVE	0X00		/*接收邮箱处于不活跃状态*/
#define	CAN_MB_RX_EMPTY		0X04		/*接收邮箱处于空闲状态,等待接收CAN信息*/
#define	CAN_MB_RX_FULL		0X02		/*接收邮箱满状态,可以读取邮箱中的内容*/
#define	CAN_MB_RX_OVERRUN		0X06		/*接收邮箱数据覆盖模式,接收数据有丢失*/
/*==========================================CAN消息邮箱TX命令宏定义=======================================*/
#define	CAN_MB_TX_INACTIVE	0X08		/*发送邮箱处于不活跃状态*/
#define	CAN_MB_TX_ABORT		0X09		/*发送邮箱处于终止发送状态*/
#define	CAN_MB_TX_DATA		0X0C		/*发送邮箱处于发送状态,无条件发送数据一次*/
/*==========================================CAN报文缓冲区格式==============================================*/
struct  	CAN_MESSAGE_BUFFER
{
unsigned int 	NOUSE2:16,DLC:4,RTR:1,IDE:1,SRR:1,NOUSE1:1,CODE:4,NOUSE:4;	/*CAN0通讯控制字*/
union
{
struct {unsigned int	ID:29,NOUSE3:3;}EXTEND;	/*CAN_ID扩展帧*/
struct {unsigned int	NOUSE4:16,NOUSE5:2,ID:11,NOUSE6:3;}STAND;	/*CAN_ID表准帧*/      
}CAN_ID;
unsigned char	Data[8];	/*CAN报文数据*/
};
/*===============================================全局变量声明区============================================*/
extern 	struct CAN_MESSAGE_BUFFER   Can0TranBuffer[];               /*CAN0发送缓冲去*/
extern 	struct CAN_MESSAGE_BUFFER   Can1TranBuffer[];               /*CAN1发送缓冲去*/
extern 	struct CAN_MESSAGE_BUFFER   Can2TranBuffer[];               /*CAN2发送缓冲去*/
extern 	struct CAN_MESSAGE_BUFFER   Can0ReceBuffer[];               /*CAN0接收缓冲区*/
extern 	struct CAN_MESSAGE_BUFFER   Can1ReceBuffer[];               /*CAN1接收缓冲区*/
extern 	struct CAN_MESSAGE_BUFFER   Can2ReceBuffer[];               /*CAN2接收缓冲区*/
extern 	int   Can0TranTop,Can0TranTail;           /*CAN0发送缓冲区头尾指针*/
extern 	int   Can1TranTop,Can1TranTail;           /*CAN1发送缓冲区头尾指针*/
extern 	int   Can2TranTop,Can2TranTail;           /*CAN2发送缓冲区头尾指针*/
extern 	int   Can0RecePosition,Can1RecePosition,Can2RecePosition;   /*Can0、Can1、Can2接收缓冲区指针位置*/
extern 	unsigned int	HostCommandState;					/*主机发送命令状态*/
extern 	unsigned int	SystemTicksPrev;					/*系统时间节拍缓冲*/
extern 	unsigned char	CcpDataBuff[];					/*CCP标定数据缓冲区*/
extern 	unsigned char	HostData[];						/*上位机发送给数据*/
extern 	unsigned	int	UbVoltage;						/*UB电压值，精度0.01V*/
extern 	unsigned	int	Ad2428Vout2;					/*AD2428输出电压Vout2电压值，精度0.01V*/
extern 	unsigned	int	Ad2428Power;					/*AD2428供电电压值，精度0.01V*/


/*=================================================函数声明区==============================================*/
extern 	void  SystemCoreClockConfigure(void);				            /*系统时钟配置函数*/
extern 	void	InitHardware(void);						            /*初始化系统硬件*/
extern 	void 	EnableCanClock(CANCHANNEL);					            /*使能CAN总线时钟*/
extern 	void 	ResetCanModule(CANCHANNEL);					            /*使CAN模块进入复位状态*/
extern	void 	InitCan(CANCHANNEL,CANBAUD);					            /*CAN0初始化函数*/
extern 	int  	TranCan(CANCHANNEL,struct  CAN_MESSAGE_BUFFER *);	            /*发送CAN数据*/
extern 	int	ReadCanData(CANCHANNEL,struct  CAN_MESSAGE_BUFFER *);             /*读取CAN报文缓冲区接收数据*/
extern 	int   CANTranQueue(CANCHANNEL,struct  CAN_MESSAGE_BUFFER *);            /*队列发送CAN数据*/
extern 	void  DmaInitial(void);                                                 /*DMA初始化模块*/
extern 	unsigned char  EepromQuickWrite(unsigned int);                          /*EEPROM快速写入*/
extern      void  FlexramEeprom(void);                                              /*设置FLEXRAM为EEPROM仿真模式*/
extern      void  FlexRamParden(void);                                              /*FLEXRAM分区命令*/
extern      void  EraseFlashSector(unsigned int);                                   /*擦除BaseAddress地址范围的Flash扇区(4K)*/
extern      void  WriteFlash(unsigned int,unsigned char *);                         /*在FLASH给定地址编程*/
extern      void  WriteEepromData(unsigned int *,unsigned int  *,unsigned int);     /*写入EEPROM数据(32位)*/
extern      void  WriteEepromShort(unsigned short *,unsigned short  *,unsigned int);/*写入EEPROM数据(16位)*/
extern      void  InitLPSpi(SPICHANNEL,unsigned int );                              /*初始化QSPI模块*/
extern      void  InitATD(void);                                                    /*InitATD初始化A/D转换*/
extern      void  StartAtdConver(void);                                             /*批量启动A/D转换*/
extern      void  GetAtdResult(void);                                               /*获得A/D转换结果*/
extern      void  AutoCalibrationATD(void);                                         /*设置ADC自动校正功能*/
extern      void  InitFtm2Capture(void);                                            /*初始化PWM特征捕捉功能*/
extern      void  StartFtm2Capture(unsigned int);                                   /*开始捕捉PWM输入特征*/
extern      void  ReadFtm2CaptureData(unsigned int);                                /*读PWM特征数据*/
extern      void  TransLPSpi(SPICHANNEL,unsigned char *,unsigned char,unsigned char,unsigned char,unsigned char);       /*发送LPSPI数据*/
extern      void  ReadLPSpi(SPICHANNEL,unsigned char *,unsigned char);              /*读LPSPI接收数据*/
extern      unsigned int      ClearLPSpiReceFIFO(SPICHANNEL);                       /*清空LPSPI接收缓冲区(FIFO)*/
extern      void EepromSpiDma(unsigned char *,unsigned char *,int);                 /*通过DMA方式与外置EEPROM进行SPI通讯*/
extern      void FTM0_init(void);                                                   /*FTM0初始化*/
extern      void FTM1_init(void);                                                   /*FTM1初始化*/
extern      void StartFtmCounter (unsigned	int);						/*FTM模块开始工作*/
extern      void FTM0_PWM_init(unsigned int,unsigned int);					/*初始化FTM0 PWM配置*/
extern      void FTM1_PWM_init(unsigned int,unsigned int);					/*初始化FTM1 PWM配置*/
extern      void	FlexIOInit(void);									/*初始化FLEX_IO -->IIS*/
extern      void	DriverMusicOutput(unsigned char *,unsigned char *,unsigned int);	/*司机座椅声音播放*/




extern      void Fmt0Dma0Tran(unsigned char *,unsigned int);				/*FMT0，DMA0数据传输*/
extern      void Fmt0Dma1Tran(unsigned char *,unsigned int);				/*FMT0，DMA1数据传输*/

extern      void FlexioDmaRece(unsigned char *,int);						/*Flex_IO，DMA12数据传输*/
extern      unsigned	int	GetIisDmaPosition(unsigned int);				/*返回DMA接收IIS数据在缓冲区中的位置*/
extern      unsigned	int	GetSoundDrvPosition(unsigned int);				/*返回DMA声音驱动数据在缓冲区中的位置*/	
extern      void	DisableDma(unsigned int);							/*关闭相应的DMA通道*/



