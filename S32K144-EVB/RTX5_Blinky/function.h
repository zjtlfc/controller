

/*===============================================================�궨����========================================================*/
/*=====================================������������غ궨��=======================================================*/
#define	IIS_RECE_BUFF_SIZE		1000			/*IIS���ݽ��ջ�������С*/
#define	SOUND_DRIVE_BUFF_SIZE		2000			/*�����������ݻ�������С*/
#define	SOUND_DATA_BUFF_SIZE		1000			/*��Ӳ���롢�����������С��16mS���ڣ�*/
/*=====================================BMS CAN���߱���ID==========================================================*/
#define     BMU_TO_BCU_CONTROL            0X20              /*BMU�����������ݸ�BCU������BCU����,ID:0X20*/
#define     BMS_RUN_MESSAGE               0X012D11EB        /*BMSϵͳ����״̬��Ϣ,ID=0X012D11EB(19730923)*/
#define     CELL_STATUS_MESSAGE           0X012DFBE8        /*��ذ���о״̬��Ϣ,ID=0X012DFBE8(19790824)*/
#define     BMS_AUX1_MESSAGE              0X013218FA        /*BMS���и�����Ϣ1,ID=0X013218FA(20060410)*/
#define     BMS_AUX2_MESSAGE              0X01282D2E        /*BMS���и�����Ϣ2,ID=0X01282D2E(19410222)*/
#define     BMS_ERROR_MESSAGE             0X012EE511        /*BMS����CAN���ϱ�������Ϣ,ID=0X012EE511(19410222)*/
#define     BMS_CHARGE_MESSAGE            0X0129DAD7        /*BMS����������Ϣ�������,ID=0X0129DAD7(19520215)*/
#define     BMS_BALANCE_MESSAGE           0X012E4AC3        /*BMSϵͳ��̬ƽ�������Ϣ,ID=0X012E4AC3(19811011)*/
#define     BMU_TO_SLOW_CHANGR            0X1806E5F4        /*BMU���͸������Ĳ�������,ID:0X1806E5F4*/
#define     BMS_INTRE_TEST_MESSAGE        0X18FF0000        /*BMS�ڲ�������CAN����,���ݸ�ʽ����,ID:0X18FF0000*/
/*======================================��CAN������غ궨��===================================*/
#define     CAN_REPEAT_BUFFER_SIZE        4    /*CANѭ���������ݻ���������*/
//#define     CHARGE_CAN                    CanChannel2       /*���CANΪCANͨ��2*/
//#define     BATTERY_CAN                   CanChannel1       /*BMS�ڲ�CANΪCANͨ��1*/
#define     SYSTEM_CAN                    CanChannel0       /*����CANΪCANͨ��0*/
/*==================================�ظ�����CAN���ݽṹ======================================*/
struct  CanRepeatTran                             
{
unsigned int      RepeatTimes;                  /*CAN���ݷ���ѭ�����ʱ��,��λmS*/
unsigned int      SurplusTimes;                 /*��ǰCAN���ݷ��͵�ʣ��ʱ��,��λmSmS*/
unsigned int      CanID;                        /*��ǰҪ����CAN���ݵ�ID��Ϣ*/
unsigned char     DataLength;                   /*��ǰҪ����CAN���ݳ���*/
unsigned char     Action;                       /*Ҫ��CANͨѶ������ִ�еĶ���0:�޶���   0X55:CAN����ǰִ�����ݸ���  0XAA:CAN���ͺ�ִ�����ݸ���*/
unsigned char     CanIdType;                    /*CAN�������ͣ�0����׼֡��1����չ֡*/
unsigned char     CanChannel;                   /*CANͨ����*/
int               Times;                        /*���͵Ĵ��� >0:�������ɴ�, =0:ֹͣ���� <0:��Զѭ������*/
unsigned char     Data[8];                      /*��ǰҪ����CAN����*/
};
/*==============================================================����������============================================================*/
extern	const	unsigned short  SoundTest1[];							/*�����������ݣ�48KHz*/
extern	struct      CanRepeatTran     CanRepeatBuff[];					/*CAN���ݷ��ͻ�����*/
extern	unsigned int	SysTickTime;							/*ϵͳ������ʱ��,����10mS*/  
extern	unsigned int	SystemTimes;    							/*��ȡϵͳ�ϵ�������ʱ��,��λS*/
extern	unsigned   int	IisDataBuffer[];							/*IIS���ݽ��ջ�����*/
extern	unsigned	int	IisReceTop,IisReceBottom;					/*IIS�������ݻ�������βָ��*/
extern	unsigned	int	ReceDataSize;							/*����Ϊ�������ݳ���*/
extern	unsigned	short		SoundOutputCH1[];					/*��ʻԱ�����������1�����ݣ�0��1666��*/
extern	unsigned	short		SoundOutputCH2[];					/*��ʻԱ�����������2�����ݣ�0��1666��*/	
extern	unsigned	int	SoundTopPoint,SoundBottomPoint;				/*���������վ绺����ͷ��βָ��*/
extern	unsigned	short	IisLeftSoundData[],IisRightSoundData[];			/*IIS��������������������*/
//extern	unsigned	short	SoundLeftOutData[],SoundRightOutData[];			/*�������������������*/
extern	unsigned	int	SoundDataLength;							/*δ���ŵ���Ӳ���ݳ���*/
extern	unsigned	int	SoundDriveMark;							/*�������ű�־��1�����ţ�0��δ����*/
extern	unsigned int	SoundDrvTop,SoundDrvBottom;					/*������������ͷ��βָ��*/	
extern	unsigned	short		SoundOutputCH3[];				/*ǰ��Ա�����������1�����ݣ�0��1666��*/
extern	unsigned	short		SoundOutputCH4[];				/*ǰ��Ա�����������2�����ݣ�0��1666��*/	
//extern	unsigned	short	SoundLeftOutData1[],SoundRightOutData1[];			/*�������������������-1*/

/*====================================================ȫ�ֺ���������============================================*/
extern      void              InitVarent(void);                                     /*��ʼ������ȫ�ֱ���*/
extern   unsigned   int InputLRData(unsigned short *,unsigned int,unsigned short *,unsigned int);		/*��ϵͳͨ��IIS���ܵ����ݷ��͵���Ӧ������������������*/
extern   unsigned   int OutputLRData(unsigned int);	/*��������������������������ݴ��䵽����������*/




