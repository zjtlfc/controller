
#define	OSC		8							/*���徧��Ϊ8MHZ*/
#define     WATCHDOG_RESET_SYSTEM   WDOG->TOVAL = (uint32_t )0;               /*�ɿ��Ź�������ϵͳ��λ*/
#define	SOFTWARE_RESET	S32_SCB->AIRCR = S32_SCB_AIRCR_VECTKEY(0x5FA) | S32_SCB_AIRCR_SYSRESETREQ(1)	/*���������ø�λ*/
#define	MASTER_COMMAND_CANAPE_CALIBRATIOB	0X00FF0000				/*ϵͳ֧��CANAPE�궨����*/
#define	MASTER_COMMAND_PARAMETER_TRANS	0X000000FF				/*ϵͳ֧�ֲ������ع���*/
#define	MASTER_COMMAND_UDS_DIAGONSE		0XFF000000				/*ϵͳ֧��UDS��Ϲ���*/
#define	MASTER_COMMAND_VARENT_UPLOAD		0X0000FF00				/*ϵͳ֧�ֱ��������ϴ�����*/
#define	RESET_VECTOR_HANDLE 			1					/*�ֶ�ǿ�ƶ�λ��λ������ַ��1��ʹ�ܣ�0����ֹ*/
#define	ATD_FILTER_SCALE				8					/*A/Dת���˲�����*/
/*===========================================������������궨��===========================================*/
#define	KL15_WAKE_SW		((PTD->PDIR & 0X00000080)>>7)      		/*KL_15Կ�׻��������ź�PTD7*/	
#define	CAN_WAKE_SW			((PTC->PDIR & 0X00000100)>>8)      		/*CAN���������ź�PTC8*/	
#define	DRIVER_DIAGON11		((PTB->PDIR & 0X00004000)>>14)		/*����ģ��11��������ź�PTB14*/
#define	DRIVER_DIAGON12		((PTB->PDIR & 0X00008000)>>15)		/*����ģ��12��������ź�PTB15*/
#define	DRIVER_DIAGON21		((PTB->PDIR & 0X00002000)>>13)		/*����ģ��21��������ź�PTB13*/
#define	DRIVER_DIAGON22		((PTB->PDIR & 0X00001000)>>12)		/*����ģ��22��������ź�PTB12*/
#define	DRIVER_DIAGON31		((PTB->PDIR & 0X00020000)>>17)		/*����ģ��31��������ź�PTB17*/
#define	DRIVER_DIAGON32		((PTB->PDIR & 0X00010000)>>16)		/*����ģ��32��������ź�PTB16*/
#define	DRIVER_DIAGON41		((PTB->PDIR & 0X00000800)>>11)		/*����ģ��41��������ź�PTB11*/
#define	DRIVER_DIAGON42		((PTB->PDIR & 0X00000100)>>8)			/*����ģ��42��������ź�PTB8*/
/*==========================================������������궨��============================================*/
#define	WATCHDOG_TRIGGER		PTC->PTOR = 0X00000200		            /*�������Ź�ιʳ,PTC9��ת*/ 
/*========================================================*/
#define	POWER_LOCK_ENABLE		PTC->PSOR = 0X00000400		            /*����ʹ�ܵ�Դ����,PTC10=1*/
#define	ENABLE_AD2428_POWER	PTE->PSOR = 0X00004000		            /*ʹ��AD2428��Դ���,PTBE14=1*/

#define	DISABLE_PTD_PD0		PTD->PCOR = 0X00000001				/*��PTD0=0*/
#define	DISABLE_PTD_PD1		PTD->PCOR = 0X00000002				/*��PTD1=0*/
#define	DISABLE_SCL_CLEAR		PTA->PCOR = 0X00000008				/*��PTA3=0*/
#define	DISABLE_SDA_CLEAR		PTA->PCOR = 0X00000004				/*��PTA2=0*/
/*========================================================*/
#define	POWER_LOCK_DISABLE	PTC->PCOR = 0X00000400		            /*���ùرյ�Դ����,PTC10=0*/
#define	DISABLE_AD2428_POWER	PTE->PCOR = 0X00004000		            /*��ֹAD2428��Դ���,PTE14=0*/
/*========================================================*/
#define	CAN_EN_ENABLE		PTE->PSOR = 0X00000002				/*ʹ��CAN_EN,PTE1=1*/
#define	CAN_STB_ENABLE		PTE->PSOR = 0X00000001				/*ʹ��CAN_STB,PTE0=1*/	
#define	CAN_EN_DISABLE		PTE->PCOR = 0X00000002				/*ʹ��CAN_EN,PTE1=0*/
#define	CAN_STB_DISABLE		PTE->PCOR = 0X00000001				/*ʹ��CAN_STB,PTE0=0*/	
/*=======================����ʻ��������������=====================*/
#define	ENABLE_STANDBY_GR1	PTD->PSOR = 0X00002000				/*��������ʻ����������STANBYģʽ��PTD13=1*/
#define	ENABLE_EN_GR1		PTD->PSOR = 0X00004000				/*��������ʻ����������ʹ�ܣ�PTD14=1*/
#define	ENABLE_FS0_GR1		PTE->PSOR = 0X00000004				/*����ʻ����Ƶ��ѡ��0ʹ�ܣ�PTE2=1*/
#define	ENABLE_FS1_GR1		PTB->PSOR = 0X00000002				/*����ʻ����Ƶ��ѡ��1ʹ�ܣ�PTB1=1*/
#define	ENABLE_FS2_GR1		PTB->PSOR = 0X00000001				/*����ʻ����Ƶ��ѡ��2ʹ�ܣ�PTB0=1*/
#define	DISABLE_STANDBY_GR1	PTD->PCOR = 0X00002000				/*�������ʻ����������STANBYģʽ��PTD13=0*/
#define	DISABLE_EN_GR1		PTD->PCOR = 0X00004000				/*��ֹ����ʻ����������ʹ�ܣ�PTD14=0*/
#define	DISABLE_FS0_GR1		PTE->PCOR = 0X00000004				/*����ʻ����Ƶ��ѡ��0��ֹ��PTE2=0*/
#define	DISABLE_FS1_GR1		PTB->PCOR = 0X00000002				/*����ʻ����Ƶ��ѡ��1��ֹ��PTB1=0*/
#define	DISABLE_FS2_GR1		PTB->PCOR = 0X00000001				/*����ʻ����Ƶ��ѡ��2��ֹ��PTB0=0*/
/*=======================ǰ�ų�Ա��������������=====================*/
#define	ENABLE_STANDBY_GR2	PTD->PSOR = 0X00000020				/*����ǰ�ų�Ա����������STANBYģʽ��PTD5=1*/
#define	ENABLE_EN_GR2		PTD->PSOR = 0X00000040				/*����ǰ�ų�Ա����������ʹ�ܣ�PTD6=1*/
#define	ENABLE_FS0_GR2		PTD->PSOR = 0X00000400				/*ǰ�ų�Ա����Ƶ��ѡ��0ʹ�ܣ�PTD10=1*/
#define	ENABLE_FS1_GR2		PTD->PSOR = 0X00000800				/*ǰ�ų�Ա����Ƶ��ѡ��1ʹ�ܣ�PTD11=1*/
#define	ENABLE_FS2_GR2		PTD->PSOR = 0X00001000				/*ǰ�ų�Ա����Ƶ��ѡ��2ʹ�ܣ�PTD12=1*/
#define	DISABLE_STANDBY_GR2	PTD->PCOR = 0X00000020				/*���ǰ�ų�Ա����������STANBYģʽ��PTD5=0*/
#define	DISABLE_EN_GR2		PTD->PCOR = 0X00000040				/*��ֹǰ�ų�Ա����������ʹ�ܣ�PTD6=0*/
#define	DISABLE_FS0_GR2		PTD->PCOR = 0X00000400				/*ǰ�ų�Ա����Ƶ��ѡ��0��ֹ��PTD10=0*/
#define	DISABLE_FS1_GR2		PTD->PCOR = 0X00000800				/*ǰ�ų�Ա����Ƶ��ѡ��1��ֹ��PTD11=0*/
#define	DISABLE_FS2_GR2		PTD->PCOR = 0X00001000				/*ǰ�ų�Ա����Ƶ��ѡ��2��ֹ��PTD12=0*/
/*=======================���ų�Ա1��������������=====================*/
#define	ENABLE_STANDBY_GR3	PTA->PSOR = 0X00020000				/*���ú��ų�Ա1����������STANBYģʽ��PTA17=1*/
#define	ENABLE_EN_GR3		PTA->PSOR = 0X00004000				/*���ú��ų�Ա1����������ʹ�ܣ�PTA14=1*/
#define	ENABLE_FS0_GR3		PTD->PSOR = 0X00000004				/*���ų�Ա1����Ƶ��ѡ��0ʹ�ܣ�PTD2=1*/
#define	ENABLE_FS1_GR3		PTD->PSOR = 0X00000008				/*���ų�Ա1����Ƶ��ѡ��1ʹ�ܣ�PTD3=1*/
#define	ENABLE_FS2_GR3		PTD->PSOR = 0X00000010				/*���ų�Ա1����Ƶ��ѡ��2ʹ�ܣ�PTD4=1*/
#define	DISABLE_STANDBY_GR3	PTA->PCOR = 0X00020000				/*������ų�Ա1����������STANBYģʽ��PTA17=0*/
#define	DISABLE_EN_GR3		PTA->PCOR = 0X00004000				/*��ֹ���ų�Ա1����������ʹ�ܣ�PTA14*/
#define	DISABLE_FS0_GR3		PTD->PCOR = 0X00000004				/*���ų�Ա1����Ƶ��ѡ��0��ֹ��PTD2=0*/
#define	DISABLE_FS1_GR3		PTD->PCOR = 0X00000008				/*���ų�Ա1����Ƶ��ѡ��1��ֹ��PTD3=0*/
#define	DISABLE_FS2_GR3		PTD->PCOR = 0X00000010				/*���ų�Ա1����Ƶ��ѡ��2��ֹ��PTD4=0*/
/*=======================���ų�Ա2��������������=====================*/
#define	ENABLE_STANDBY_GR4	PTA->PSOR = 0X00008000				/*���ú��ų�Ա2����������STANBYģʽ��PTA15=1*/
#define	ENABLE_EN_GR4		PTE->PSOR = 0X00000040				/*���ú��ų�Ա2����������ʹ�ܣ�PTE6=1*/
#define	ENABLE_FS0_GR4		PTA->PSOR = 0X00002000				/*���ų�Ա2����Ƶ��ѡ��0ʹ�ܣ�PTA13=1*/
#define	ENABLE_FS1_GR4		PTA->PSOR = 0X00001000				/*���ų�Ա2����Ƶ��ѡ��1ʹ�ܣ�PTA12=1*/
#define	ENABLE_FS2_GR4		PTA->PSOR = 0X00000800				/*���ų�Ա2����Ƶ��ѡ��2ʹ�ܣ�PTA11=1*/
#define	DISABLE_STANDBY_GR4	PTA->PCOR = 0X00008000				/*������ų�Ա2����������STANBYģʽ��PTA15=0*/
#define	DISABLE_EN_GR4		PTE->PCOR = 0X00000040				/*��ֹ���ų�Ա2����������ʹ�ܣ�PTE6*/
#define	DISABLE_FS0_GR4		PTA->PCOR = 0X00002000				/*���ų�Ա2����Ƶ��ѡ��0��ֹ��PTA13=0*/
#define	DISABLE_FS1_GR4		PTA->PCOR = 0X00001000				/*���ų�Ա2����Ƶ��ѡ��1��ֹ��PTA12=0*/
#define	DISABLE_FS2_GR4		PTA->PCOR = 0X00000800				/*���ų�Ա2����Ƶ��ѡ��2��ֹ��PTA11=0*/
/*================================================================*/
/*==================================��������˿ڵ�����״̬���=========================================*/
/*=============================================ATD0ת��ͨ������=============================================*/
#define     UB_VOLTAGE_CHANNEL         	0                             /*ϵͳ����UB��ѹA/Dת��ͨ��*/
#define     AD2428_VOUT2_CHANNEL          1                             /*AD2424�����ѹVout��ѹA/Dת��ͨ��*/
#define     AD2428_POWER_CHANNEL       	2                             /*AD 2428�����ѹA/Dת��ͨ��*/
//#define     SLOW_CHARGE_CUR_CHANNEL       3                             /*����Ӵ�����Ȧ��������A/Dת��ͨ��*/
//#define     QUICK_CHARGE_CUR_CHANNEL      4                             /*���Ӵ�����Ȧ��������A/Dת��ͨ��*/
//#define     AC_RELAY_CUR_CHANNEL          5                             /*�յ��Ӵ�����Ȧ��������A/Dת��ͨ��*/
/*=============================================ATD1ת��ͨ������=============================================*/
/*===============================================EEPROM��غ궨��===========================================*/
#define     ERASR_FLASH_SECTOR      0x09                                /*����FLASH�����е���������*/
#define     PROGRAM_FLASH           0x07                                /*��FLASH����һ��д�����*/
#define     EEPROM_DEPART           8                                   /*EEPROM��������,64KEFLASH*/
#define     EEPROM_EEESIZE          0X02                                /*EEPROM��С:4KByte*/
#define     EEPROM_PGMPART          0X80                                /*��EEPROM���з���*/
#define     EEPROM_SE_FlEXRAM       0X81                                /*����FLEXRAM����ģʽ*/
#define     EEPROM_EMULATED         0X00                                /*FLEXRAM�����ڷ���EEPROMģʽ*/
#define     EEPROM_QUERY_STATUS     0X77                                /*FLEXRAM����״̬��ѯ*/
#define     EEPROM_QUICK_WRITE      0X55                                /*FLEXRAM����д�����*/
/*=================================================���ݽṹ����=============================================*/
/*=================================================FLEXTIMERdingyi==========================================*/
#define	FMT0_DMA_SOURCE		36						/*FTM0ģ��DMAԴ*/
#define	FTM1_CH0_DMA_SROUCE	20						/*FTM1ģ��ͨ��0 DMAԴ*/
#define	FTM1_CH1_DMA_SROUCE	21						/*FTM1ģ��ͨ��1 DMAԴ*/
#define	FTM1_CH2_DMA_SROUCE	22						/*FTM1ģ��ͨ��2 DMAԴ*/
#define	FTM1_CH3_DMA_SROUCE	23						/*FTM1ģ��ͨ��3 DMAԴ*/
#define	FTM1_CH4_DMA_SROUCE	24						/*FTM1ģ��ͨ��4 DMAԴ*/
#define	FTM1_CH5_DMA_SROUCE	25						/*FTM1ģ��ͨ��5 DMAԴ*/
#define	FTM1_CH6_DMA_SROUCE	26						/*FTM1ģ��ͨ��6 DMAԴ*/
#define	FTM1_CH7_DMA_SROUCE	27						/*FTM1ģ��ͨ��7 DMAԴ*/
/*=================================================CAN�����ʶ���============================================*/
typedef	enum {CAN_1000 ,CAN_500 ,CAN_250 , CAN_200 , CAN_125 , CAN_100 , CAN_50 } CANBAUD;
/*===================================================����CAN�ں�============================================*/
typedef	enum { CanChannel0 ,CanChannel1 ,CanChannel2 } CANCHANNEL;
/*===================================================����SPIͨ����==========================================*/
typedef	enum { SpiChannel0 ,SpiChannel1 ,SpiChannel2 } SPICHANNEL;
/*==========================================================================================================*/
#define	CAN0_RECE_MN_SIZE		31					      /*CAN0�����������䳤��*/
#define	CAN0_TRAN_MN_POS		31					      /*CAN0������������λ��*/
#define	CAN1_RECE_MN_SIZE		5					      /*CAN1�����������䳤��*/
#define	CAN1_TRAN_MN_POS		5					      /*CAN1������������λ��*/
#define	CAN2_RECE_MN_SIZE		5					      /*CAN2�����������䳤��*/
#define	CAN2_TRAN_MN_POS		5					      /*CAN2������������λ��*/
#define     CAN0_TRAN_BUFF_SIZE     40                                  /*CAN0���ͻ���������*/
#define     CAN1_TRAN_BUFF_SIZE     5                                  /*CAN1���ͻ���������*/
#define     CAN2_TRAN_BUFF_SIZE     5                                  /*CAN2���ͻ���������*/
#define     CAN0_RECE_BUFF_SIZE     32                                  /*CAN0���ܻ���������*/
#define     CAN1_RECE_BUFF_SIZE     5                                  /*CAN1���ܻ���������*/
#define     CAN2_RECE_BUFF_SIZE     5                                  /*CAN2���ܻ���������*/
#define     DMA_CAN0_SOURCE         54                                  /*CAN0����DMAԴ*/
#define     DMA_CAN1_SOURCE         55                                  /*CAN1����DMAԴ*/
#define     DMA_CAN2_SOURCE         56                                  /*CAN2����DMAԴ*/

#define	DMA_FLEXIO_SHIFTER0_SOURCE	10					/*FLEX_IO��λ�Ĵ���0��DMAԴ*/
/*=========================================CAN��Ϣ����RX����궨��=========================================*/
#define	CAN_MB_RX_INACTIVE	0X00		/*�������䴦�ڲ���Ծ״̬*/
#define	CAN_MB_RX_EMPTY		0X04		/*�������䴦�ڿ���״̬,�ȴ�����CAN��Ϣ*/
#define	CAN_MB_RX_FULL		0X02		/*����������״̬,���Զ�ȡ�����е�����*/
#define	CAN_MB_RX_OVERRUN		0X06		/*�����������ݸ���ģʽ,���������ж�ʧ*/
/*==========================================CAN��Ϣ����TX����궨��=======================================*/
#define	CAN_MB_TX_INACTIVE	0X08		/*�������䴦�ڲ���Ծ״̬*/
#define	CAN_MB_TX_ABORT		0X09		/*�������䴦����ֹ����״̬*/
#define	CAN_MB_TX_DATA		0X0C		/*�������䴦�ڷ���״̬,��������������һ��*/
/*==========================================CAN���Ļ�������ʽ==============================================*/
struct  	CAN_MESSAGE_BUFFER
{
unsigned int 	NOUSE2:16,DLC:4,RTR:1,IDE:1,SRR:1,NOUSE1:1,CODE:4,NOUSE:4;	/*CAN0ͨѶ������*/
union
{
struct {unsigned int	ID:29,NOUSE3:3;}EXTEND;	/*CAN_ID��չ֡*/
struct {unsigned int	NOUSE4:16,NOUSE5:2,ID:11,NOUSE6:3;}STAND;	/*CAN_ID��׼֡*/      
}CAN_ID;
unsigned char	Data[8];	/*CAN��������*/
};
/*===============================================ȫ�ֱ���������============================================*/
extern 	struct CAN_MESSAGE_BUFFER   Can0TranBuffer[];               /*CAN0���ͻ���ȥ*/
extern 	struct CAN_MESSAGE_BUFFER   Can1TranBuffer[];               /*CAN1���ͻ���ȥ*/
extern 	struct CAN_MESSAGE_BUFFER   Can2TranBuffer[];               /*CAN2���ͻ���ȥ*/
extern 	struct CAN_MESSAGE_BUFFER   Can0ReceBuffer[];               /*CAN0���ջ�����*/
extern 	struct CAN_MESSAGE_BUFFER   Can1ReceBuffer[];               /*CAN1���ջ�����*/
extern 	struct CAN_MESSAGE_BUFFER   Can2ReceBuffer[];               /*CAN2���ջ�����*/
extern 	int   Can0TranTop,Can0TranTail;           /*CAN0���ͻ�����ͷβָ��*/
extern 	int   Can1TranTop,Can1TranTail;           /*CAN1���ͻ�����ͷβָ��*/
extern 	int   Can2TranTop,Can2TranTail;           /*CAN2���ͻ�����ͷβָ��*/
extern 	int   Can0RecePosition,Can1RecePosition,Can2RecePosition;   /*Can0��Can1��Can2���ջ�����ָ��λ��*/
extern 	unsigned int	HostCommandState;					/*������������״̬*/
extern 	unsigned int	SystemTicksPrev;					/*ϵͳʱ����Ļ���*/
extern 	unsigned char	CcpDataBuff[];					/*CCP�궨���ݻ�����*/
extern 	unsigned char	HostData[];						/*��λ�����͸�����*/
extern 	unsigned	int	UbVoltage;						/*UB��ѹֵ������0.01V*/
extern 	unsigned	int	Ad2428Vout2;					/*AD2428�����ѹVout2��ѹֵ������0.01V*/
extern 	unsigned	int	Ad2428Power;					/*AD2428�����ѹֵ������0.01V*/


/*=================================================����������==============================================*/
extern 	void  SystemCoreClockConfigure(void);				            /*ϵͳʱ�����ú���*/
extern 	void	InitHardware(void);						            /*��ʼ��ϵͳӲ��*/
extern 	void 	EnableCanClock(CANCHANNEL);					            /*ʹ��CAN����ʱ��*/
extern 	void 	ResetCanModule(CANCHANNEL);					            /*ʹCANģ����븴λ״̬*/
extern	void 	InitCan(CANCHANNEL,CANBAUD);					            /*CAN0��ʼ������*/
extern 	int  	TranCan(CANCHANNEL,struct  CAN_MESSAGE_BUFFER *);	            /*����CAN����*/
extern 	int	ReadCanData(CANCHANNEL,struct  CAN_MESSAGE_BUFFER *);             /*��ȡCAN���Ļ�������������*/
extern 	int   CANTranQueue(CANCHANNEL,struct  CAN_MESSAGE_BUFFER *);            /*���з���CAN����*/
extern 	void  DmaInitial(void);                                                 /*DMA��ʼ��ģ��*/
extern 	unsigned char  EepromQuickWrite(unsigned int);                          /*EEPROM����д��*/
extern      void  FlexramEeprom(void);                                              /*����FLEXRAMΪEEPROM����ģʽ*/
extern      void  FlexRamParden(void);                                              /*FLEXRAM��������*/
extern      void  EraseFlashSector(unsigned int);                                   /*����BaseAddress��ַ��Χ��Flash����(4K)*/
extern      void  WriteFlash(unsigned int,unsigned char *);                         /*��FLASH������ַ���*/
extern      void  WriteEepromData(unsigned int *,unsigned int  *,unsigned int);     /*д��EEPROM����(32λ)*/
extern      void  WriteEepromShort(unsigned short *,unsigned short  *,unsigned int);/*д��EEPROM����(16λ)*/
extern      void  InitLPSpi(SPICHANNEL,unsigned int );                              /*��ʼ��QSPIģ��*/
extern      void  InitATD(void);                                                    /*InitATD��ʼ��A/Dת��*/
extern      void  StartAtdConver(void);                                             /*��������A/Dת��*/
extern      void  GetAtdResult(void);                                               /*���A/Dת�����*/
extern      void  AutoCalibrationATD(void);                                         /*����ADC�Զ�У������*/
extern      void  InitFtm2Capture(void);                                            /*��ʼ��PWM������׽����*/
extern      void  StartFtm2Capture(unsigned int);                                   /*��ʼ��׽PWM��������*/
extern      void  ReadFtm2CaptureData(unsigned int);                                /*��PWM��������*/
extern      void  TransLPSpi(SPICHANNEL,unsigned char *,unsigned char,unsigned char,unsigned char,unsigned char);       /*����LPSPI����*/
extern      void  ReadLPSpi(SPICHANNEL,unsigned char *,unsigned char);              /*��LPSPI��������*/
extern      unsigned int      ClearLPSpiReceFIFO(SPICHANNEL);                       /*���LPSPI���ջ�����(FIFO)*/
extern      void EepromSpiDma(unsigned char *,unsigned char *,int);                 /*ͨ��DMA��ʽ������EEPROM����SPIͨѶ*/
extern      void FTM0_init(void);                                                   /*FTM0��ʼ��*/
extern      void FTM1_init(void);                                                   /*FTM1��ʼ��*/
extern      void StartFtmCounter (unsigned	int);						/*FTMģ�鿪ʼ����*/
extern      void FTM0_PWM_init(unsigned int,unsigned int);					/*��ʼ��FTM0 PWM����*/
extern      void FTM1_PWM_init(unsigned int,unsigned int);					/*��ʼ��FTM1 PWM����*/
extern      void	FlexIOInit(void);									/*��ʼ��FLEX_IO -->IIS*/
extern      void	DriverMusicOutput(unsigned char *,unsigned char *,unsigned int);	/*˾��������������*/




extern      void Fmt0Dma0Tran(unsigned char *,unsigned int);				/*FMT0��DMA0���ݴ���*/
extern      void Fmt0Dma1Tran(unsigned char *,unsigned int);				/*FMT0��DMA1���ݴ���*/

extern      void FlexioDmaRece(unsigned char *,int);						/*Flex_IO��DMA12���ݴ���*/
extern      unsigned	int	GetIisDmaPosition(unsigned int);				/*����DMA����IIS�����ڻ������е�λ��*/
extern      unsigned	int	GetSoundDrvPosition(unsigned int);				/*����DMA�������������ڻ������е�λ��*/	
extern      void	DisableDma(unsigned int);							/*�ر���Ӧ��DMAͨ��*/



