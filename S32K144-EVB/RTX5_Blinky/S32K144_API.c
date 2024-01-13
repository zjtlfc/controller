#include "S32K144_API.h"
#include "system_S32K144.h"             // Keil::Device:Startup
#include "clock_S32K1xx.h"              // Keil::Device:SDK Driver:Clock
#include "clock_manager.h"              // Keil::Device:SDK Driver:Clock
#include "string.h"
//#include "S32K144.h"
#include "function.h"
#include "s32_core_cm4.h"
/*===========================================================特殊常量定义区===========================================================*/
#if	RESET_VECTOR_HANDLE == 1		/*手动强制定位复位向量地址*/
extern	void Reset_Handler(void);
const	unsigned int	PushPollPointer __attribute__((at(0x0000))) = 0X20007000;					/*定义上电初始化堆栈指针*/
const	unsigned int	ResetVectorPROC __attribute__((at(0x0004))) =(unsigned int)Reset_Handler;		/*定义上电初始化首地址*/
#endif
/*===============================================================全局变量定义区=======================================================*/
struct CAN_MESSAGE_BUFFER   Can0TranBuffer[CAN0_TRAN_BUFF_SIZE];              /*CAN0发送缓冲去*/
struct CAN_MESSAGE_BUFFER   Can1TranBuffer[CAN1_TRAN_BUFF_SIZE];              /*CAN1发送缓冲去*/
struct CAN_MESSAGE_BUFFER   Can2TranBuffer[CAN2_TRAN_BUFF_SIZE];              /*CAN2发送缓冲去*/
struct CAN_MESSAGE_BUFFER   Can0ReceBuffer[CAN0_RECE_BUFF_SIZE];              /*CAN0接收缓冲区*/
struct CAN_MESSAGE_BUFFER   Can1ReceBuffer[CAN1_RECE_BUFF_SIZE];              /*CAN1接收缓冲区*/
struct CAN_MESSAGE_BUFFER   Can2ReceBuffer[CAN2_RECE_BUFF_SIZE];              /*CAN2接收缓冲区*/
int   Can0TranTop,Can0TranTail;                                               /*CAN0发送缓冲区头尾指针*/
int   Can1TranTop,Can1TranTail;                                               /*CAN1发送缓冲区头尾指针*/
int   Can2TranTop,Can2TranTail;                                               /*CAN2发送缓冲区头尾指针*/
int   Can0RecePosition,Can1RecePosition,Can2RecePosition;                     /*Can0、Can1、Can2接收缓冲区指针位置*/
unsigned int	HostCommandState;								/*主机发送命令状态*/
unsigned int	SystemTicksPrev;								/*系统时间节拍缓冲*/
unsigned	char	HostData[8];								/*上位机发送给数据*/	
unsigned	char	CcpDataBuff[8];								/*CCP标定数据缓冲区*/
unsigned	int	Atd0Pos;									/*当前A/D转换通道号*/
unsigned	int	UbVoltageValue,Ad2428Vout2Value,AD2428PowerValue;		/*三路电压监控A/D转换值*/
int	UbVoltageBuff,Ad2428Vout2Buff,Ad2428PowerBuff;					/*A/D转换滤波缓冲区*/
unsigned	int	UbVoltage;									/*UB电压值，精度0.01V*/
unsigned	int	Ad2428Vout2;								/*AD2428输出电压Vout2电压值，精度0.01V*/
unsigned	int	Ad2428Power;								/*AD2428供电电压值，精度0.01V*/
/*----------------------------------------------------------------------------
  configure SystemCoreClock:
     System Clock source         PLL (OSC)
     OSC Frequency(Hz)           8000000
     SYSCLK(Hz)                  80000000
     PLLMUL                      20
     PREDIV                      2
 *----------------------------------------------------------------------------*/
void SystemCoreClockConfigure(void) 
{

	SCG->SOSCCSR = SCG_SOSCCSR_SOSCERR(1U);		/*系统时钟监控使能(外部晶体振荡器)*/

	/* set up OSC clock. */
	/* Setup dividers. */
	SCG->SOSCDIV = (SCG_SOSCDIV_SOSCDIV1(1U) |
                  SCG_SOSCDIV_SOSCDIV2(1U)  );	/*系统OSC时钟倍除因子*/

	/* Set OSC configuration. */
	SCG->SOSCCFG = (SCG_SOSCCFG_EREFS(1U) |
                  SCG_SOSCCFG_RANGE(2U)  );	/*使用石英晶体振荡器、频率为中等速度*/

	/* Enable clock. */
	SCG->SOSCCSR = (SCG_SOSCCSR_SOSCEN(1U) );	/*使能系统石英振荡器*/

	/* Wait for OSC clock to be valid. */
	while (!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD(1U))) __NOP();	/*等待检测石英振荡器工作稳定*/

	/* Init SIRC */
	SCG->SIRCCSR = 0U;	/*所有Slow IRC相关功能关闭*/

	/* Setup dividers. */
	SCG->SIRCDIV = (SCG_SIRCDIV_SIRCDIV1(1U) |
                  SCG_SIRCDIV_SIRCDIV2(2U)  );	/*Slow IRC倍除因子*/

	/* Set SIRC configuration. */
	SCG->SIRCCFG = SCG_SIRCCFG_RANGE(1U);	/*设置Slow IRC频率为8MHz*/

	/* Enable clock. */
	SCG->SIRCCSR = SCG_SIRCCSR_SIRCEN(1U);	/*使能Slow IRC时钟*/

	/* Wait for SIRC clock to be valid. */
	while (!(SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD(1U))) __NOP();	/*等待Slow IRC工作稳定*/

	/* Change to SIRC as system clock source */
	SCG->RCCR = (SCG_RCCR_DIVSLOW(3U) |
               SCG_RCCR_DIVBUS(0U)  |
               SCG_RCCR_DIVCORE(0U) |
               SCG_RCCR_SCS(2U)     );	/*设置系统时钟为Slow IRC、总线时钟倍除因子为1、核时钟倍除因子为1、Slow Clock倍除因子为3*/

	/* Wait for clock source switch finished. */
	while ((SCG->RCCR & 0x0F000000) != 0x02000000) __NOP();;	/*等待时钟切换完成*/

	/* Init Firc */
	SCG->FIRCCSR = 0U;	/*所有Fast IRC相关功能关闭*/

	/* Setup dividers. */
	SCG->FIRCDIV = (SCG_FIRCDIV_FIRCDIV1(1U) |
                  SCG_FIRCDIV_FIRCDIV2(1U)  );	/*Fast IRC倍除因子*/

	/* Set FIRC configuration. */
	SCG->FIRCCFG = SCG_FIRCCFG_RANGE(0U);	/*设置Fast IRC频率为48MHz*/

	/* Enable clock. */
	SCG->FIRCCSR |= SCG_FIRCCSR_FIRCEN(1U);	/*使能Fast IRC*/

	/* Wait for FIRC clock to be valid. */
	while (!(SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD(1U))) __NOP();;	/*等待Fast IRC工作稳定*/

	/* Init SysPll */
	SCG->SPLLCSR = SCG_SPLLCSR_SPLLERR(1U);		/*系统锁相环监控使能*/

	/* Setup dividers. */
	SCG->SPLLDIV = (SCG_SPLLDIV_SPLLDIV1(1U) |
                  SCG_SPLLDIV_SPLLDIV2(2U)  );	/*设置锁相环倍除因子*/

	/* Set PLL configuration. */
	SCG->SPLLCFG = (SCG_SPLLCFG_PREDIV(0U) |                     /* prediv 1 */
                  SCG_SPLLCFG_MULT(4U)   );                    /* pll multiply 20 */

	/* Enable clock. */
	SCG->SPLLCSR = SCG_SPLLCSR_SPLLEN(1U);	/*使能锁相环时钟*/

	/* Wait for PLL clock to be valid. */
	while (!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD(1U))) __NOP();	/*等待锁相环时钟工作稳定*/

	/* Change to use PLL as system clock source */
	SCG->RCCR = (SCG_RCCR_DIVSLOW(2U) |
               SCG_RCCR_DIVBUS(1U)  |
               SCG_RCCR_DIVCORE(0U) |
               SCG_RCCR_SCS(6U)     );	/*系统使用锁相环输出时钟、总线倍除因子为2、核倍除因子为1、Slow Clock倍除因子为3*/

	/* Wait for clock source switch finished. */
	while ((SCG->RCCR & 0x0F000000) != 0x06000000) __NOP();	/*等待时钟切换稳定*/
}
/*=================================================初始化系统硬件====================================================*/
void	InitHardware(void)
{
	/*=================================使能所有外设时钟============================================*/
	PCC->PCCn[PCC_PORTA_INDEX]=PCC_PCCn_CGC(1);	/*端口A时钟使能*/
	PCC->PCCn[PCC_PORTB_INDEX]=PCC_PCCn_CGC(1);	/*端口B时钟使能*/
	PCC->PCCn[PCC_PORTC_INDEX]=PCC_PCCn_CGC(1);	/*端口C时钟使能*/
	PCC->PCCn[PCC_PORTD_INDEX]=PCC_PCCn_CGC(1);	/*端口D时钟使能*/
	PCC->PCCn[PCC_PORTE_INDEX]=PCC_PCCn_CGC(1);	/*端口E时钟使能*/
//      SIM->PLATCGC=SIM->PLATCGC | SIM_PLATCGC_CGCDMA(1);             /*DMA通道时钟默认为启动*/
      PCC->PCCn[PCC_DMAMUX_INDEX] = PCC_PCCn_CGC(1);  /*使能DMAMUX时钟*/
      PCC->PCCn[PCC_FTFC_INDEX]=PCC_PCCn_CGC(1);	/*FTFC-FLASH时钟使能*/
	
	/*=============================设置端口功能====================================================*/
	PORTA->PCR[5]=PORT_PCR_MUX(7);			/*PTA5为复位输入口*/
	PORTA->PCR[17]=PORT_PCR_MUX(1);			/*PTA17为一般I/O口输出*/
	PORTA->PCR[16]=PORT_PCR_MUX(1);			/*PTA16为一般I/O口输出*/
	PORTA->PCR[15]=PORT_PCR_MUX(1);			/*PTA15为一般I/O口输出*/
	PORTA->PCR[14]=PORT_PCR_MUX(1);			/*PTA14为一般I/O口输出*/
	PORTA->PCR[13]=PORT_PCR_MUX(1);			/*PTA13为一般I/O口输出*/
	PORTA->PCR[12]=PORT_PCR_MUX(1);			/*PTA12为一般I/O口输出*/
	PORTA->PCR[11]=PORT_PCR_MUX(1);			/*PTA11为一般I/O口输出*/
	PORTA->PCR[3]=PORT_PCR_MUX(1);			/*PTA3为一般I/O口输出*/
	PORTA->PCR[2]=PORT_PCR_MUX(1);			/*PTA2为一般I/O口输出*/
	
	PORTB->PCR[17]=PORT_PCR_MUX(1);			/*PTB17为一般I/O口输出*/
	PORTB->PCR[16]=PORT_PCR_MUX(1);			/*PTB16为一般I/O口输出*/
	PORTB->PCR[15]=PORT_PCR_MUX(1);			/*PTB15为一般I/O口输出*/
	PORTB->PCR[14]=PORT_PCR_MUX(1);			/*PTB14为一般I/O口输出*/
	PORTB->PCR[13]=PORT_PCR_MUX(1);			/*PTB13为一般I/O口输出*/
	PORTB->PCR[12]=PORT_PCR_MUX(1);			/*PTB12为一般I/O口输出*/
	PORTB->PCR[11]=PORT_PCR_MUX(1);			/*PTB11为一般I/O口输出*/
	PORTB->PCR[8]=PORT_PCR_MUX(1);			/*PTB8为一般I/O口输出*/
	PORTB->PCR[5]=PORT_PCR_MUX(2);			/*PTB5为FTM0_CH5输出*/
	PORTB->PCR[4]=PORT_PCR_MUX(2);			/*PTB4为FTM0_CH4输出*/
	PORTB->PCR[3]=PORT_PCR_MUX(2);			/*PTB3为FTM1_CH1输出*/
	PORTB->PCR[2]=PORT_PCR_MUX(2);			/*PTB2为FTM1_CH0输出*/
	PORTB->PCR[1]=PORT_PCR_MUX(1);			/*PTB1为一般I/O口输出*/
	PORTB->PCR[0]=PORT_PCR_MUX(1);			/*PTB0为一般I/O口输出*/
	
	PORTC->PCR[17]=PORT_PCR_MUX(1);			/*PTC17为一般I/O口输出*/
	PORTC->PCR[16]=PORT_PCR_MUX(1);			/*PTC16为一般I/O口输出*/
	PORTC->PCR[15]=PORT_PCR_MUX(2);			/*PTC15FTM1_CH3输出*/
	PORTC->PCR[14]=PORT_PCR_MUX(2);			/*PTC14FTM1_CH2输出*/
	PORTC->PCR[10]=PORT_PCR_MUX(1);			/*PTC10为一般I/O口输出*/
	PORTC->PCR[9]=PORT_PCR_MUX(1);			/*PTC9为一般I/O口输出*/
	PORTC->PCR[8]=PORT_PCR_MUX(1);			/*PTC8为一般I/O口输出*/
	PORTC->PCR[3]=PORT_PCR_MUX(2);			/*PTC3为FTM0_CH3输出*/
	PORTC->PCR[2]=PORT_PCR_MUX(2);			/*PTC2为FTM0_CH2输出*/
	PORTC->PCR[1]=PORT_PCR_MUX(6);			/*PTC1为FTM1_CH7输出*/
	PORTC->PCR[0]=PORT_PCR_MUX(6);			/*PTC0为FTM1_CH6输出*/
	
	PORTD->PCR[17]=PORT_PCR_MUX(1);			/*PTD17为一般I/O口输出*/
	PORTD->PCR[16]=PORT_PCR_MUX(2);			/*PTD16为FTM0_CH1输出*/
	PORTD->PCR[15]=PORT_PCR_MUX(2);			/*PTD15为FTM0_CH0输出*/
	PORTD->PCR[14]=PORT_PCR_MUX(1);			/*PTD14为一般I/O口输出*/
	PORTD->PCR[13]=PORT_PCR_MUX(1);			/*PTD13为一般I/O口输出*/
	PORTD->PCR[12]=PORT_PCR_MUX(1);			/*PTD12为一般I/O口输出*/
	PORTD->PCR[11]=PORT_PCR_MUX(1);			/*PTD11为一般I/O口输出*/
	PORTD->PCR[10]=PORT_PCR_MUX(1);			/*PTD10为一般I/O口输出*/
	PORTD->PCR[9]=PORT_PCR_MUX(6);			/*PTD9为FTM1_CH5输出*/
	PORTD->PCR[8]=PORT_PCR_MUX(6);			/*PTD8为FTM1_CH4输出*/
	PORTD->PCR[7]=PORT_PCR_MUX(1);			/*PTD7为一般I/O口输出*/
	PORTD->PCR[6]=PORT_PCR_MUX(1);			/*PTD6为一般I/O口输出*/
	PORTD->PCR[5]=PORT_PCR_MUX(1);			/*PTD5为一般I/O口输出*/
	PORTD->PCR[4]=PORT_PCR_MUX(1);			/*PTD4为一般I/O口输出*/
	PORTD->PCR[3]=PORT_PCR_MUX(1);			/*PTD3为一般I/O口输出*/
	PORTD->PCR[2]=PORT_PCR_MUX(1);			/*PTD2为一般I/O口输出*/
	PORTD->PCR[1]=PORT_PCR_MUX(1);			/*PTD1为一般I/O口输出*/
	PORTD->PCR[0]=PORT_PCR_MUX(1);			/*PTD0为一般I/O口输出*/
//	PORTD->PCR[1]=PORT_PCR_MUX(6);			/*PTD1为FXIO_D1输出*/
//	PORTD->PCR[0]=PORT_PCR_MUX(6);			/*PTD0为FXIO_D0输出*/
	
	PORTE->PCR[16]=PORT_PCR_MUX(6);			/*PTE16为FXIO_D3输出*/
	PORTE->PCR[15]=PORT_PCR_MUX(6);			/*PTE15为FXIO_D2输出*/
	PORTE->PCR[14]=PORT_PCR_MUX(1);			/*PTE14为一般I/O口输出*/
	PORTE->PCR[11]=PORT_PCR_MUX(6);			/*PTE11为FXIO_D5输出*/
	PORTE->PCR[10]=PORT_PCR_MUX(6);			/*PTE10为FXIO_D4输出*/
	PORTE->PCR[9]=PORT_PCR_MUX(2);			/*PTE9为FXIO_D7输出*/
	PORTE->PCR[8]=PORT_PCR_MUX(2);			/*PTE8为FXIO_D6输出*/
	PORTE->PCR[6]=PORT_PCR_MUX(1);			/*PTE6为一般I/O口输出*/
	PORTE->PCR[2]=PORT_PCR_MUX(1);			/*PTE2为一般I/O口输出*/
	PORTE->PCR[1]=PORT_PCR_MUX(1);			/*PTE1为一般I/O口输出*/
	PORTE->PCR[0]=PORT_PCR_MUX(1);			/*PTE0为一般I/O口输出*/
	/*==========================IIC总线端口配置========================================*/
//	PORTA->PCR[2]=PORT_PCR_MUX(3);			/*PTA2为IIC_SDA输出*/
//	PORTA->PCR[3]=PORT_PCR_MUX(3);			/*PTA3为IIC_SCL输出*/
	/*==========================CAN总线端口配置========================================*/
	PORTE->PCR[4]=PORT_PCR_MUX(5);			/*设置PTD4为CAN0_RX通讯口*/
	PORTE->PCR[5]=PORT_PCR_MUX(5);			/*设置PTD5为CAN0_TX通讯口*/
	
//	PORTA->PCR[12]=PORT_PCR_MUX(3);			/*设置PTA12为CAN1_RX通讯口*/
//	PORTA->PCR[13]=PORT_PCR_MUX(3);			/*设置PTA13为CAN1_TX通讯口*/
	
//	PORTC->PCR[16]=PORT_PCR_MUX(3);			/*设置PTC16位CAN2_RX通讯口*/
//	PORTC->PCR[17]=PORT_PCR_MUX(3);			/*设置PTC17位CAN2_TX通讯口*/
      /*==========================LPSPI端口配置==========================================*/
//      PORTB->PCR[4]=PORT_PCR_MUX(3);                  /*LPSPI模块SPI0通道(SOUT)*/
//      PORTB->PCR[5]=PORT_PCR_MUX(4);                  /*LPSPI模块SPI0通道(PCS0)*/
//      PORTD->PCR[15]=PORT_PCR_MUX(4);                 /*LPSPI模块SPI0通道(SCK)*/
//      PORTD->PCR[16]=PORT_PCR_MUX(4);                 /*LPSPI模块SPI0通道(SIN)*/
      
//      PORTD->PCR[1]=PORT_PCR_MUX(3);                  /*LPSPI模块SPI1通道(SIN)*/
//      PORTD->PCR[0]=PORT_PCR_MUX(3);                  /*LPSPI模块SPI1通道(PCS0)*/
//      PORTE->PCR[1]=PORT_PCR_MUX(5);                  /*LPSPI模块SPI1通道(SCK)*/
//      PORTB->PCR[16]=PORT_PCR_MUX(3);                 /*LPSPI模块SPI1通道(SOUT)*/

//      PORTE->PCR[11]=PORT_PCR_MUX(2);                 /*LPSPI模块SPI2通道(PCS0)*/
//      PORTE->PCR[15]=PORT_PCR_MUX(3);                 /*LPSPI模块SPI2通道(SCK)*/
//      PORTE->PCR[16]=PORT_PCR_MUX(3);                 /*LPSPI模块SPI2通道(SIN)*/
//      PORTA->PCR[8]=PORT_PCR_MUX(3);                  /*LPSPI模块SPI2通道(SOUT)*/
//      PORTE->PCR[10]=PORT_PCR_MUX(3);                 /*LPSPI模块SPI2通道(PCS1)*/
      /*============================A/D转换模块端口配置====================================*/
      PORTA->PCR[0]=PORT_PCR_MUX(0);                  /*ADC0_SE0*/
      PORTA->PCR[1]=PORT_PCR_MUX(0);                  /*ADC0_SE1*/
      PORTA->PCR[6]=PORT_PCR_MUX(0);                  /*ADC0_SE2*/
//      PORTA->PCR[7]=PORT_PCR_MUX(0);                  /*ADC0_SE3*/
//      PORTB->PCR[0]=PORT_PCR_MUX(0);                  /*ADC0_SE4*/
//      PORTB->PCR[1]=PORT_PCR_MUX(0);                  /*ADC0_SE5*/
//      PORTB->PCR[2]=PORT_PCR_MUX(0);                  /*ADC0_SE6*/
//      PORTB->PCR[3]=PORT_PCR_MUX(0);                  /*ADC0_SE7*/
//      PORTC->PCR[0]=PORT_PCR_MUX(0);                  /*ADC0_SE8*/
//      PORTC->PCR[1]=PORT_PCR_MUX(0);                  /*ADC0_SE9*/
//      PORTC->PCR[2]=PORT_PCR_MUX(0);                  /*ADC0_SE10*/
//      PORTC->PCR[3]=PORT_PCR_MUX(0);                  /*ADC0_SE11*/
//      PORTD->PCR[2]=PORT_PCR_MUX(0);                  /*ADC1_SE2*/
//      PORTD->PCR[3]=PORT_PCR_MUX(0);                  /*ADC1_SE3*/
//      PORTD->PCR[4]=PORT_PCR_MUX(0);                  /*ADC1_SE4*/
//      PORTC->PCR[6]=PORT_PCR_MUX(0);                  /*ADC1_SE5*/
//      PORTC->PCR[7]=PORT_PCR_MUX(0);                  /*ADC1_SE6*/
//      PORTB->PCR[12]=PORT_PCR_MUX(0);                 /*ADC1_SE7*/
//      PORTB->PCR[13]=PORT_PCR_MUX(0);                 /*ADC1_SE8*/
//      PORTB->PCR[14]=PORT_PCR_MUX(0);                 /*ADC1_SE9*/
//      PORTE->PCR[2]=PORT_PCR_MUX(0);                  /*ADC1_SE10*/
//      PORTE->PCR[6]=PORT_PCR_MUX(0);                  /*ADC1_SE11*/
//      PORTA->PCR[15]=PORT_PCR_MUX(0);                 /*ADC1_SE12*/
//      PORTA->PCR[16]=PORT_PCR_MUX(0);                 /*ADC1_SE13*/
      /*=================================输入捕捉端口设置======================================*/
//      PORTD->PCR[10]=PORT_PCR_MUX(2);                  /*FTM2_CH0输入捕捉端口*/
	/*==================================端口方向设定===========================================*/
	PTA->PDDR=0X3F80C;					/*PTA17、16、15、14、13、12、11、3、2为输出口*/
	PTB->PDDR=0X003F;						/*PTB0、1、2、3、4、5为输出端口*/
	PTC->PDDR=0XC70F;						/*PTC10、9、8、14、15、0、1、2、3为输出端口*/
	PTD->PDDR=0X3FF7F;					/*PTD17—PTD8，PTD6—PTD0为输出口*/
	PTE->PDDR=0X4347;						/*PTE0、1、2、6、8、9、14为输出口*/
      
	DISABLE_SCL_CLEAR;					/*PTA3=0*/
	DISABLE_SDA_CLEAR;					/*PTA2=0*/
	DISABLE_PTD_PD0;						/*PTD0=0*/
	DISABLE_PTD_PD1;						/*PTD1=0*/
	
	POWER_LOCK_ENABLE;					/*设置使能电源自锁,PTC10=1*/

      /*================================InitATD初始化A/D转换===================================*/
	Atd0Pos=UB_VOLTAGE_CHANNEL;		/*当前A/D转换通道号:系统供电UB电压A/D转换通道*/
	UbVoltageValue=0;				/*UB电压AD转换值*/
	Ad2428Vout2Value=0;			/*AD2428输出电压Vout的A/D转换值*/
	AD2428PowerValue=0;			/*AD2428供电电压A/D转换值*/
	UbVoltageBuff=0;				/*A/D转换滤波缓冲区*/
	Ad2428Vout2Buff=0;			/*A/D转换滤波缓冲区*/
	Ad2428PowerBuff=0;			/*A/D转换滤波缓冲区*/
      InitATD();
      AutoCalibrationATD();                           /*ADC自动校正*/    
      WATCHDOG_TRIGGER;       /*触发看门狗喂食,PTC9翻转*/ 
      /*===============================初始化PWM特征捕捉功能=====================================*/
	FTM0_init();				/*初始化FTM0模块*/	
	FTM1_init();				/*初始化FTM1模块*/	
//      InitFtm2Capture();                                             
      /*===================================初始化DMA模块=======================================*/
      DmaInitial();
	/*==================================初始换CAN总线模块=====================================*/
	CAN_EN_ENABLE;			/*使能CAN_EN,PTE1=1*/
	CAN_STB_ENABLE;			/*使能CAN_STB,PTE0=1*/	
	InitCan(CanChannel0,CAN_500);				/*CAN0初始化，速率500Khz，CAN0为整车CAN通道*/
      /*===================================初始化LPSpi模块======================================*/
//      InitLPSpi(SpiChannel0,1);                       /*初始化QSPI0模块，使能DMA传输*/
//      InitLPSpi(SpiChannel1,1);                       /*初始化QSPI1模块，使能DMA传输*/
//      InitLPSpi(SpiChannel2,0);                       /*初始化QSPI2模块，禁止DMA传输*/
      /*======================================FLEXRAM分区命令====================================*/
      WATCHDOG_TRIGGER;       /*触发看门狗喂食,PTC9翻转*/ 
      FlexRamParden();                                        
      /*=================================设置FLEXRAM为EEPROM仿真模式=============================*/
      WATCHDOG_TRIGGER;       /*触发看门狗喂食,PTC9翻转*/ 
      FlexramEeprom();              
      /*====================================中断向量重新定位====================================*/
      S32_SCB->VTOR=0X8000;                           /*中断向量表重新定位到0X8000处,底端FLASH留给Bootloader用*/
}
/*============================================使能CAN总线时钟====================================
	返回值:NULL
	参数:Channel是一个枚举变量，表示CAN总线的通道号
================================================================================================*/	
void 	EnableCanClock(CANCHANNEL Channel)
{
	switch(Channel)
	{
		case	CanChannel0:
			PCC->PCCn[PCC_FlexCAN0_INDEX]=PCC_PCCn_CGC(1);	/*使能CAN0时钟源*/
		break;
		case	CanChannel1:
			PCC->PCCn[PCC_FlexCAN1_INDEX]=PCC_PCCn_CGC(1);	/*使能CAN1时钟源*/
		break;
		case	CanChannel2:
			PCC->PCCn[PCC_FlexCAN2_INDEX]=PCC_PCCn_CGC(1);	/*使能CAN2时钟源*/
		break;
	};
};
/*=====================================使CAN模块进入复位状态=====================================
	返回值:NULL
	参数:Channel是一个枚举变量，表示CAN总线的通道号
================================================================================================*/
void 	ResetCanModule(CANCHANNEL Channel)
{
	switch(Channel)
	{
		case	CanChannel0:
			CAN0->MCR=CAN0->MCR | CAN_MCR_SOFTRST(1);		/*CAN0进入CAN总线复位模式*/
			while(CAN0->MCR & CAN_MCR_SOFTRST(1)) {};		/*等待CAN总线复位完成*/
		break;
		case	CanChannel1:
			CAN1->MCR=CAN1->MCR | CAN_MCR_SOFTRST(1);		/*CAN0进入CAN总线复位模式*/
			while(CAN1->MCR & CAN_MCR_SOFTRST(1)) {};		/*等待CAN总线复位完成*/
		break;
		case	CanChannel2:
			CAN2->MCR=CAN2->MCR | CAN_MCR_SOFTRST(1);		/*CAN0进入CAN总线复位模式*/
			while(CAN2->MCR & CAN_MCR_SOFTRST(1)) {};		/*等待CAN总线复位完成*/
		break;
	};
};
/*=====================================================初始化CAN总线==========================================
	返回值：无
	参  数：
		Channel：CAN模块号，这是一个CANCHANNEL变量，表示CAN的三个模块{CanChannel0 ,CanChannel1 ,CanChannel2}
		baud：CAN通讯速率，这是一个CANBAUD变量，{CAN_1000,CAN_500,CAN_250,CAN_200,CAN_125,CAN_100,CAN_50}
=============================================================================================================*/
void 	InitCan(CANCHANNEL Channel,CANBAUD baud)				/*CAN0初始化函数*/
{
CAN_Type *CanPoint;
unsigned char	PreDivide;		/*时钟预分频数*/	
unsigned char	ReceMnSize;		/*邮箱位置,接收邮箱数量,发送邮箱位置*/
unsigned char	TranMnPos;	
unsigned char	MnPos;
	switch(Channel)
	{
		case	CanChannel0:
			CanPoint=(CAN_Type *)CAN0_BASE;			/*设置相应的CAN指针首地址*/
			ReceMnSize=CAN0_RECE_MN_SIZE;				/*CAN0接收数据邮箱长度*/
			TranMnPos=CAN0_TRAN_MN_POS;				/*CAN0发送数据邮箱位置*/
		break;
		case	CanChannel1:
			CanPoint=(CAN_Type *)CAN1_BASE;			/*设置相应的CAN指针首地址*/
			ReceMnSize=CAN1_RECE_MN_SIZE;				/*CAN1接收数据邮箱长度*/
			TranMnPos=CAN1_TRAN_MN_POS;				/*CAN1发送数据邮箱位置*/
		break;
		case	CanChannel2:
			CanPoint=(CAN_Type *)CAN2_BASE;			/*设置相应的CAN指针首地址*/
			ReceMnSize=CAN2_RECE_MN_SIZE;				/*CAN2接收数据邮箱长度*/
			TranMnPos=CAN2_TRAN_MN_POS;				/*CAN2发送数据邮箱位置*/
		break;
	};
	switch(baud)
	{
		case	CAN_1000:
			PreDivide=0;	/*预分频倍数为1*/
		break;
		case	CAN_500:
			PreDivide=1;	/*预分频倍数为2*/
		break;
		case	CAN_250:
			PreDivide=3;	/*预分频倍数为4*/
		break;
		case	CAN_200:
			PreDivide=4;	/*预分频倍数为5*/
		break;
		case	CAN_125:
			PreDivide=7;	/*预分频倍数为8*/
		break;
		case	CAN_100:
			PreDivide=9;	/*预分频倍数为10*/
		break;
		case	CAN_50:
			PreDivide=19;	/*预分频倍数为20*/
		break;
		default:
			PreDivide=1;	/*默认预分频倍数为2*/
		break;
	};
      /*==================================使能CAN0总线时钟========================================*/
	EnableCanClock(Channel);				            
	/*============================================================配置MCR==============================================================*/
      /*=================================Disable CAN模块===========================================*/
      if(((CanPoint->MCR & CAN_MCR_MDIS_MASK) >> CAN_MCR_MDIS_SHIFT) == 0U)
      {
            CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_MDIS_MASK) | CAN_MCR_MDIS(1U);
            while (((CanPoint->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT) == 0U) {}
      }
      if((CanPoint->MCR & CAN_MCR_LPMACK_MASK) == 0U)
      {
            CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_MDIS_MASK) | CAN_MCR_MDIS(1U);
            while (((CanPoint->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT) == 0U) {}
      };
	/*================================CAN进入CAN总线复位模式====================================*/  
		CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_SOFTRST_MASK) | CAN_MCR_SOFTRST(1U);		
		while (((CanPoint->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) != 0U) {}
		while ((CanPoint->MCR & CAN_MCR_SOFTRST_MASK)==0) {};      /*等待复位完成*/
      /*===================================使能CAN模块=============================================*/      
      if(((CanPoint->MCR & CAN_MCR_MDIS_MASK) >> CAN_MCR_MDIS_SHIFT))
      {
            CanPoint->MCR=CanPoint->MCR & 0X7FFFFFFF;				
            while (((CanPoint->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT)) {};     
      };           
      /*=================================CAN模块进入Freeze模式====================================*/
      CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(1U);
      if((CanPoint->MCR & CAN_MCR_HALT_MASK)==0)
      {
            CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_HALT_MASK) | CAN_MCR_HALT(1U);
            while (((CanPoint->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) == 0U) {}
      };
      
      /*================================接收缓冲区=================================================*/
      CanPoint->RXMGMASK=0;
      CanPoint->RX14MASK=0;
      
      CanPoint->MCR=CanPoint->MCR | CAN_MCR_SRXDIS(1);            /*关闭发送,接收自循环*/

      CanPoint->RX15MASK=0;      
      CanPoint->IMASK1=0;                                         /*关闭MB中断*/
	/*============================================清除消息缓冲区===============================================*/
	MnPos=0;								/*初始值为0*/
	while(MnPos<(ReceMnSize + 1)*4)		/*初始化接收邮箱*/
	{
		CanPoint->RAMn[MnPos]=0X0;					      /*不激活状态*/
		MnPos++;
	};
	/*=========================================================================================================*/
      CanPoint->MCR=CanPoint->MCR | CAN_MCR_RFEN(1);              /*CAN总线接收FIFO模式*/
      CanPoint->RXFGMASK=0;                                       /*FIFO全局屏蔽寄存器*/
      CanPoint->MCR=CanPoint->MCR | CAN_MCR_DMA(1);               /*CAN总线接收DMA模式*/
      
	CanPoint->MCR=(CanPoint->MCR & 0XFFFFFF00)+ (ReceMnSize+1);	/*MB邮箱数量*/
	/*===============配置CTRL1===============*/
	CanPoint->CTRL1=CAN_CTRL1_PRESDIV(PreDivide) | CAN_CTRL1_PSEG2(2) | CAN_CTRL1_PSEG1(2) | CAN_CTRL1_LBUF(1);
	/*===============配置CBT==================*/
	CanPoint->CBT=CAN_CBT_EPSEG2(2) | CAN_CBT_EPSEG1(2)| CAN_CBT_EPRESDIV(PreDivide);
	/*================================CAN模块离开Freeze模式================================*/
      CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(0U);
      while (((CanPoint->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) != 0U) {}      
	/*================================配置CAN总线发送/接收邮箱=============================*/
      /*============================================设置接收邮箱===============================================*/
//	MnPos=0;								/*初始值为0*/
//	while(MnPos<ReceMnSize)		/*初始化接收邮箱*/
//	{
//		CanPoint->RAMn[MnPos*4]=0X0;					      /*不激活状态*/
//            CanPoint->RAMn[MnPos*4+1]=0X1FFFFFFF;
//		CanPoint->RAMn[MnPos*4]=0X04000000;					/*定义为接受缓冲区，活跃状态*/
//            CanPoint->RXIMR[MnPos]=0;                                   /*接收掩码寄存器*/
//		MnPos++;
//	};
//      CanPoint->RAMn[0]=0X04000000;	
//      CanPoint->RXIMR[0]=0;                                   /*接收掩码寄存器*/
//      PreDivide=CanPoint->TIMER;             /*接收邮箱解锁*/
//	MnPos=1;								/*初始值为0*/
//	while(MnPos<ReceMnSize)		/*初始化接收邮箱*/
//	{
//		CanPoint->RAMn[MnPos*4]=0X0;					      /*不激活状态*/
//            CanPoint->RAMn[MnPos*4+1]=0XFFFFFFFF;
//		CanPoint->RAMn[MnPos*4]=0X04000000;					/*定义为接受缓冲区，活跃状态*/
//		MnPos++;
//	};
//      CanPoint->RAMn[24]=0;
//      CanPoint->RAMn[25]=0;
//      CanPoint->RAMn[26]=0;
//      CanPoint->RAMn[27]=0;
//      CanPoint->RAMn[28]=0;
//      CanPoint->RAMn[29]=0;
//      CanPoint->RAMn[30]=0;
//      CanPoint->RAMn[31]=0;
//      /*=========================================设置发送邮箱位置===============================================*/	
	CanPoint->RAMn[TranMnPos*4]=0X08;					/*定义为发送缓冲区，不活跃状态*/
};
/*====================================================发送CAN数据================================================
	返回值:
            0：发送成功。
            -1：发送缓冲区忙，无法进行本次发送。
	参数:
		Channel：CAN模块号，这是一个CANCHANNEL变量，表示CAN的三个模块{CanChannel0 ,CanChannel1 ,CanChannel2}
		CanData:CAN报文缓冲区数据结构.
================================================================================================================*/
int  	TranCan(CANCHANNEL Channel,struct  	CAN_MESSAGE_BUFFER *CanData)
{
unsigned int	TransPos;		/*CAN发送邮箱位置*/
unsigned int	TransCode;		/*发送邮箱命令代码*/	
CAN_Type *CanPoint;
	switch(Channel)
	{
		case	CanChannel0:
			CanPoint=(CAN_Type *)CAN0_BASE;			/*设置相应的CAN指针首地址*/
			TransPos=CAN0_TRAN_MN_POS;				/*CAN0发送数据邮箱位置*/
		break;
		case	CanChannel1:
			CanPoint=(CAN_Type *)CAN1_BASE;			/*设置相应的CAN指针首地址*/
			TransPos=CAN1_TRAN_MN_POS;				/*CAN1发送数据邮箱位置*/
		break;
		case	CanChannel2:
			CanPoint=(CAN_Type *)CAN2_BASE;			/*设置相应的CAN指针首地址*/
			TransPos=CAN2_TRAN_MN_POS;				/*CAN2发送数据邮箱位置*/
		break;
	};
	TransCode=((CanPoint->RAMn[TransPos * 4]>> 24) & 0X0F );	/*得出当前邮箱的命令代码*/
	if((TransCode==CAN_MB_TX_INACTIVE) || (TransCode==CAN_MB_RX_INACTIVE))	/*发送邮箱处于不活跃状态,可以发送数据*/
	{
		CanData->CODE=CAN_MB_TX_DATA;		/*发送邮箱处于发送状态,无条件发送数据一次*/
            CanPoint->RAMn[TransPos*4+2]=CAN_RAMn_DATA_BYTE_3(CanData->Data[3])+CAN_RAMn_DATA_BYTE_2(CanData->Data[2])+CAN_RAMn_DATA_BYTE_1(CanData->Data[1])+CAN_RAMn_DATA_BYTE_0(CanData->Data[0]);
            CanPoint->RAMn[TransPos*4+3]=CAN_RAMn_DATA_BYTE_3(CanData->Data[7])+CAN_RAMn_DATA_BYTE_2(CanData->Data[6])+CAN_RAMn_DATA_BYTE_1(CanData->Data[5])+CAN_RAMn_DATA_BYTE_0(CanData->Data[4]);
		(void)memcpy((unsigned char *)&(CanPoint->RAMn[TransPos * 4]),(unsigned char *)CanData,8);
            return 0;
	}else
      {     
            return -1;
      }
};
/*==================================================读取CAN报文缓冲区接收数据=======================================
	返回值：
            0：CAN接收缓冲区中没有数据。
            1：CAN接收缓冲区中有有效的CAN数据。
	参数：
		Channel：CAN模块号，这是一个CANCHANNEL变量，表示CAN的三个模块{CanChannel0 ,CanChannel1 ,CanChannel2}
		CanData:CAN报文缓冲区数据结构。
==================================================================================================================*/
int	ReadCanData(CANCHANNEL Channel,struct  CAN_MESSAGE_BUFFER *CanData)
{
struct CAN_MESSAGE_BUFFER *CanPoint;
int	ReceMbSize;		      /*CAN接收邮箱长度*/ 
int	MbPosition;	            /*DMA接受CAN缓冲区位置*/
int   *TempReadPosition;      /*当前读缓接收冲区位置*/
	switch(Channel)
	{
		case	CanChannel0:
			CanPoint=Can0ReceBuffer;			/*设置相应的接收缓冲区首地址*/
			ReceMbSize=CAN0_RECE_BUFF_SIZE;	      /*CAN0收缓冲长度*/
                  MbPosition=CAN0_RECE_BUFF_SIZE - (DMA->TCD[15].CITER.ELINKNO);      /*DMA通道当前写入接收缓冲区位置*/
                  TempReadPosition=&Can0RecePosition;
		break;
		case	CanChannel1:
			CanPoint=Can1ReceBuffer;			/*设置相应的接收缓冲区首地址*/
			ReceMbSize=CAN1_RECE_BUFF_SIZE;	      /*CAN1收缓冲长度*/
                  MbPosition=CAN1_RECE_BUFF_SIZE - (DMA->TCD[14].CITER.ELINKNO);      /*DMA通道当前写入接收缓冲区位置*/
                  TempReadPosition=&Can1RecePosition;
		break;
		case	CanChannel2:
			CanPoint=Can2ReceBuffer;			/*设置相应的接收缓冲区首地址*/
			ReceMbSize=CAN2_RECE_BUFF_SIZE;	      /*CAN2收缓冲长度*/
                  MbPosition=CAN2_RECE_BUFF_SIZE - (DMA->TCD[13].CITER.ELINKNO);      /*DMA通道当前写入接收缓冲区位置*/
                  TempReadPosition=&Can2RecePosition;
		break;
	};
	if(*TempReadPosition != MbPosition)		/*接收缓冲区接收到新的数据*/
	{
                  memcpy(CanData , (unsigned char *)(&CanPoint[*TempReadPosition]) , 8);   /*取出控制子与ID信息*/
                  CanData->Data[0]=CanPoint[*TempReadPosition].Data[3];
                  CanData->Data[1]=CanPoint[*TempReadPosition].Data[2];
                  CanData->Data[2]=CanPoint[*TempReadPosition].Data[1];
                  CanData->Data[3]=CanPoint[*TempReadPosition].Data[0];
                  CanData->Data[4]=CanPoint[*TempReadPosition].Data[7];
                  CanData->Data[5]=CanPoint[*TempReadPosition].Data[6];
                  CanData->Data[6]=CanPoint[*TempReadPosition].Data[5];
                  CanData->Data[7]=CanPoint[*TempReadPosition].Data[4];
                  (*TempReadPosition)++;     /*指针移动一位*/
                  if(*TempReadPosition >= ReceMbSize) *TempReadPosition=0;  /*接受缓冲区为循环队列*/   
                  return 1;
	}else       /*接收缓冲区没有收到新的数据*/
      {
            return 0;
      };
};
/*=====================================================队列发送CAN数据========================================================
	返回值:
            0：发送成功。
            -1：发送缓冲区忙，无法进行本次发送。
	参数:
		Channel：CAN模块号，这是一个CANCHANNEL变量，表示CAN的三个模块{CanChannel0 ,CanChannel1 ,CanChannel2}
		CanData:CAN报文缓冲区数据结构.
============================================================================================================================*/
int      CANTranQueue(CANCHANNEL Channel,struct  CAN_MESSAGE_BUFFER *CanData)
{
int  *TempTopPoint,*TempTailPoint;
int   TempSize,TempValue;
struct CAN_MESSAGE_BUFFER     *TempBuff;
      DISABLE_INTERRUPTS();         /*关闭任务调度*/
	switch(Channel)
	{
		case	CanChannel0:                        /*CAN0发送缓冲区头尾指针*/
			TempTopPoint=&Can0TranTop;
			TempTailPoint=&Can0TranTail;
                  TempSize=CAN0_TRAN_BUFF_SIZE;       /*CAN0发送缓冲区长度*/
                  TempBuff=Can0TranBuffer;
		break;
		case	CanChannel1:                        /*CAN1发送缓冲区头尾指针*/
			TempTopPoint=&Can1TranTop;
			TempTailPoint=&Can1TranTail;
                  TempSize=CAN1_TRAN_BUFF_SIZE;       /*CAN1发送缓冲区长度*/
                  TempBuff=Can1TranBuffer;
		break;
		case	CanChannel2:                        /*CAN2发送缓冲区头尾指针*/
			TempTopPoint=&Can2TranTop;
			TempTailPoint=&Can2TranTail;
                  TempSize=CAN2_TRAN_BUFF_SIZE;       /*CAN2发送缓冲区长度*/
                  TempBuff=Can2TranBuffer;
		break;
	};
      TempValue=*TempTailPoint + 1;
      if(TempValue>=TempSize) TempValue=0;      /*循环队列*/
      if(TempValue != *TempTopPoint)      /*发送缓冲区有有效的位置*/
      {
            *TempTailPoint=TempValue;     /*修改发送队列尾指针*/
            memcpy((unsigned char *)&TempBuff[TempValue],(unsigned char *)CanData,16);    /*将要发送的数据拷贝到发送缓冲区中*/
            ENABLE_INTERRUPTS();          /*使能任务调度*/
            return 0;
      }else
      {
            ENABLE_INTERRUPTS();          /*使能任务调度*/
            return -1;  /*发送缓冲区已经满,发送数据失败*/
      }
      
};
/*======================================================DMA初始化模块======================================================*/
void DmaInitial(void)
{
      /*======================================设置DMAMUX通道,并使能=======================================*/
      DMAMUX->CHCFG[15]=0;          /*DMA通道15为CAN0接收FIFO,使能DMA通道15*/
	
	
      DMAMUX->CHCFG[12]=0;          /*DMA通道12为FLEX_IO接收,使能DMA通道12*/
	
	
      DMAMUX->CHCFG[0]=0;          /*DMA通道0为驾驶员座椅驱动数据接收,使能DMA通道0*/
      DMAMUX->CHCFG[1]=0;          /*DMA通道1为驾驶员座椅驱动数据接收,使能DMA通道1*/

      DMAMUX->CHCFG[2]=0;          /*DMA通道2为前乘员座椅驱动数据接收,使能DMA通道2*/
      DMAMUX->CHCFG[3]=0;          /*DMA通道3为前乘员座椅驱动数据接收,使能DMA通道3*/


      DMAMUX->CHCFG[15]=DMAMUX_CHCFG_ENBL(1) | DMA_CAN0_SOURCE;         /*DMA通道15为CAN0接收FIFO,使能DMA通道15*/

      DMAMUX->CHCFG[12]=DMAMUX_CHCFG_ENBL(1) | DMA_FLEXIO_SHIFTER0_SOURCE;         /*DMA通道12为FLEX_IO接收,使能DMA通道12*/
	
      DMAMUX->CHCFG[0]=DMAMUX_CHCFG_ENBL(1) | FMT0_DMA_SOURCE;         /*DMA通道0为驾驶员座椅驱动数据接收,使能DMA通道0*/
      DMAMUX->CHCFG[1]=DMAMUX_CHCFG_ENBL(1) | FTM1_CH1_DMA_SROUCE;     /*DMA通道1为驾驶员座椅驱动数据接收,使能DMAFTM1模块通道1 DMA源*/

      DMAMUX->CHCFG[3]=DMAMUX_CHCFG_ENBL(1) | FTM1_CH5_DMA_SROUCE;     /*DMA通道1为驾驶员座椅驱动数据接收,使能DMAFTM1模块通道7 DMA源*/
      DMAMUX->CHCFG[2]=DMAMUX_CHCFG_ENBL(1) | FTM1_CH7_DMA_SROUCE;     /*DMA通道1为驾驶员座椅驱动数据接收,使能DMAFTM1模块通道5 DMA源*/

//      DMAMUX->CHCFG[12]=DMAMUX_CHCFG_ENBL(1) | DMA_SPI0_RECE_SOURCE;    /*SPI0通道RX DMA源*/
//      DMAMUX->CHCFG[11]=DMAMUX_CHCFG_ENBL(1) | DMA_SPI0_TRAN_SOURCE;    /*SPI0通道TX DMA源*/
      
//      DMAMUX->CHCFG[10]=DMAMUX_CHCFG_ENBL(1) | DMA_SPI1_RECE_SOURCE;    /*SPI0通道RX DMA源*/
//      DMAMUX->CHCFG[9]= DMAMUX_CHCFG_ENBL(1) | DMA_SPI1_TRAN_SOURCE;    /*SPI0通道TX DMA源*/
      
      /*===========================================设置EDMA参数==========================================*/
      DMA->CR=DMA_CR_CLM(1) | DMA_CR_ERCA(1) | DMA_CR_EMLM(1);  /*小循环使能*/
      
      /*=========================================EDMA设置初始化==============================================*/
      DMA->DCHPRI[15]=DMA_DCHPRI_CHPRI(15)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*CAN0通道DMA优先级*/

      DMA->DCHPRI[12]=DMA_DCHPRI_CHPRI(12)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*FLEX_IO接收通道DMA优先级*/

      DMA->DCHPRI[0]=DMA_DCHPRI_CHPRI(0)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*驾驶员座椅驱动通道DMA优先级*/
      DMA->DCHPRI[1]=DMA_DCHPRI_CHPRI(1)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*驾驶员座椅驱动通道DMA优先级*/
      DMA->DCHPRI[2]=DMA_DCHPRI_CHPRI(2)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*驾驶员座椅驱动通道DMA优先级*/
      DMA->DCHPRI[3]=DMA_DCHPRI_CHPRI(3)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*驾驶员座椅驱动通道DMA优先级*/
	
//      DMA->DCHPRI[12]=DMA_DCHPRI_CHPRI(12)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*SPI0 RECE通道DMA优先级*/
//      DMA->DCHPRI[11]=DMA_DCHPRI_CHPRI(11)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*SPI0 TRAN通道DMA优先级*/
//      DMA->DCHPRI[10]=DMA_DCHPRI_CHPRI(10)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*SPI1 RECE通道DMA优先级*/
//      DMA->DCHPRI[9] =DMA_DCHPRI_CHPRI(9)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);          /*SPI1 TRAN通道DMA优先级*/
      
      /*================================设置DMA通道15的TCB配置(CAN0)=====================================*/
      DMA->TCD[15].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(CAN0_RECE_BUFF_SIZE);        /*在主循环内共计有CAN0_RECE_BUFF_SIZE个小循环*/
      DMA->TCD[15].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(CAN0_RECE_BUFF_SIZE);        /*在主循环内共计有CAN0_RECE_BUFF_SIZE个小循环*/
      DMA->TCD[15].NBYTES.MLOFFYES=DMA_TCD_NBYTES_MLOFFYES_NBYTES(16)|DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16)|DMA_TCD_NBYTES_MLOFFYES_SMLOE(1);          /*DMA传输一次16个字节*/
      DMA->TCD[15].SADDR=DMA_TCD_SADDR_SADDR(&CAN0->RAMn[0]);           /*CAN0通道DMA传输源地址*/ 
      DMA->TCD[15].SOFF=4;                           /*DMA一次读写4个字节*/
      DMA->TCD[15].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,32位数据格式*/
      DMA->TCD[15].SLAST=DMA_TCD_SLAST_SLAST(-16);    /*主循环完成后源地址回调*/
      DMA->TCD[15].DADDR=DMA_TCD_DADDR_DADDR(&Can0ReceBuffer[0]);
      DMA->TCD[15].DOFF=4;                           /*DMA一次读写4个字节*/
      DMA->TCD[15].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-16*CAN0_RECE_BUFF_SIZE);     /*主循环完成后目标地址回调*/
      DMA->TCD[15].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0);
      
//      /*================================设置DMA通道14的TCB配置(CAN1)=====================================*/
//      DMA->TCD[14].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(CAN1_RECE_BUFF_SIZE);         /*在主循环内共计有CAN1_RECE_BUFF_SIZE个小循环*/
//      DMA->TCD[14].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(CAN1_RECE_BUFF_SIZE);         /*在主循环内共计有CAN1_RECE_BUFF_SIZE个小循环*/
//      DMA->TCD[14].NBYTES.MLOFFYES=DMA_TCD_NBYTES_MLOFFYES_NBYTES(16)|DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16)|DMA_TCD_NBYTES_MLOFFYES_SMLOE(1);      /*DMA传输一次16个字节*/
//      DMA->TCD[14].SADDR=DMA_TCD_SADDR_SADDR(&CAN1->RAMn[0]);           /*CAN0通道DMA传输源地址*/ 
//      DMA->TCD[14].SOFF=4;                           /*DMA一次读写4个字节*/
//      DMA->TCD[14].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,32位数据格式*/
//      DMA->TCD[14].SLAST=DMA_TCD_SLAST_SLAST(-16);    /*主循环完成后源地址回调*/
//      DMA->TCD[14].DADDR=DMA_TCD_DADDR_DADDR(&Can1ReceBuffer[0]);
//      DMA->TCD[14].DOFF=4;                           /*DMA一次读写4个字节*/
//      DMA->TCD[14].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-16*CAN1_RECE_BUFF_SIZE);     /*主循环完成后目标地址回调*/
//      DMA->TCD[14].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0);
//      
//      /*================================设置DMA通道13的TCB配置(CAN2)=====================================*/
//      DMA->TCD[13].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(CAN2_RECE_BUFF_SIZE);        /*在主循环内共计有CAN2_RECE_BUFF_SIZE个小循环*/     
//      DMA->TCD[13].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(CAN2_RECE_BUFF_SIZE);        /*在主循环内共计有CAN2_RECE_BUFF_SIZE个小循环*/
//      DMA->TCD[13].NBYTES.MLOFFYES=DMA_TCD_NBYTES_MLOFFYES_NBYTES(16)|DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16)|DMA_TCD_NBYTES_MLOFFYES_SMLOE(1);          /*DMA传输一次16个字节*/
//      DMA->TCD[13].SADDR=DMA_TCD_SADDR_SADDR(&CAN2->RAMn[0]);           /*CAN0通道DMA传输源地址*/ 
 //     DMA->TCD[13].SOFF=4;                           /*DMA一次读写4个字节*/
 //     DMA->TCD[13].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,32位数据格式*/
 //     DMA->TCD[13].SLAST=DMA_TCD_SLAST_SLAST(-16);    /*主循环完成后源地址回调*/
  //    DMA->TCD[13].DADDR=DMA_TCD_DADDR_DADDR(&Can2ReceBuffer[0]);
 //     DMA->TCD[13].DOFF=4;                           /*DMA一次读写4个字节*/
 //     DMA->TCD[13].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-16*CAN2_RECE_BUFF_SIZE);     /*主循环完成后目标地址回调*/
 //     DMA->TCD[13].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0);
            


      DMA->SERQ=DMA_SERQ_SERQ(15);        /*使能CAN0通道DMA服务*/
//      DMA->SERQ=DMA_SERQ_SERQ(14);        /*使能CAN1通道DMA服务*/
//      DMA->SERQ=DMA_SERQ_SERQ(13);        /*使能CAN2通道DMA服务*/


};
#pragma arm section code = "RAMCODE"
/*=================================================EEPROM快速写入=====================================================*/
unsigned char  EepromQuickWrite(unsigned int Size)
{
      if(((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK)>>SIM_FCFG1_DEPART_SHIFT)== EEPROM_DEPART)/*如果EEPROM没有分区代码,64KEFLASH*/
      {     /*===========EEPROM没有进行分区操作,对EEPROM进行分区==============*/
            /*======================设置FLEXRAM工作在仿真EEPROM模式================================================*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*等待上一次FLASH操作完成*/
            FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*清除相关报警*/
            FTFC->FCCOB[3]=EEPROM_SE_FlEXRAM;   /*设置FLEXRAM工作模式FCCOB0*/
            FTFC->FCCOB[2]=EEPROM_QUICK_WRITE;  /*FLEXRAM快速写入操作,FCCoB1*/
            FTFC->FCCOB[1]=0;
            FTFC->FCCOB[0]=0;
            FTFC->FCCOB[7]=Size>>8;
            FTFC->FCCOB[6]=Size & 0XFF;
            FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*开始执行FLASH擦除命令,写1清除*/
            while(((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) || ((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK) ==0)) __NOP();    /*等待FLASH操作完成*/
      };
      return FTFC->FCCOB[6];
};
/*======================================================FLEXRAM分区命令=============================================*/
void  FlexRamParden(void)
{
      if(((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK)>>SIM_FCFG1_DEPART_SHIFT)!= EEPROM_DEPART)/*如果EEPROM没有分区代码,64KEFLASH*/
      {     /*===========EEPROM没有进行分区操作,对EEPROM进行分区==============*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*等待上一次FLASH操作完成*/
            FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*清除相关报警*/
            FTFC->FCCOB[3]=EEPROM_PGMPART;      /*对EEPROM进行分区FCCOB0*/
            FTFC->FCCOB[2]=0;                   /*FCCOB1*/
            FTFC->FCCOB[1]=0;                   /*FCCOB2*/
            FTFC->FCCOB[0]=1;                   /*FCCOB3,系统复位后不用自动装载EEPROM数据*/
            FTFC->FCCOB[7]=EEPROM_EEESIZE;      /*EEPROM大小:2KByte,FCCOB4*/
            FTFC->FCCOB[6]=EEPROM_DEPART;       /*EEPROM分区代码,64KEFLASH,FCCOB5*/
            FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*开始执行FLASH擦除命令,写1清除*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  {};    /*等待命令完成*/
      };
};
/*========================================擦除BaseAddress地址范围的Flash扇区(4K)=================================================*/
void EraseFlashSector(unsigned int  BaseAddress)
{
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*等待上一次FLASH操作完成*/
      FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*清除相关报警*/
      FTFC->FCCOB[3]=ERASR_FLASH_SECTOR;        /*擦除FLASH扇区中的所有数据*/
      FTFC->FCCOB[2]=(unsigned char)(BaseAddress>>16);
      FTFC->FCCOB[1]=(unsigned char)(BaseAddress>>8);
      FTFC->FCCOB[0]=(unsigned char)BaseAddress;
      FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*开始执行FLASH擦除命令,写1清除*/
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*等待命令完成*/
};
/*===========================================在FLASH给定地址编程=============================================================*/
void  WriteFlash(unsigned int  FlashAddress,unsigned char *ReceDataBuff)
{
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*等待上一次FLASH操作完成*/
      FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*清除相关报警*/
      FTFC->FCCOB[3]=PROGRAM_FLASH;                               /*对FLASH进行一次写入程序*/
      FTFC->FCCOB[2]=(unsigned char)(FlashAddress>>16);
      FTFC->FCCOB[1]=(unsigned char)(FlashAddress>>8);
      FTFC->FCCOB[0]=(unsigned char)FlashAddress;
      FTFC->FCCOB[4]=ReceDataBuff[0];
      FTFC->FCCOB[5]=ReceDataBuff[1];
      FTFC->FCCOB[6]=ReceDataBuff[2];
      FTFC->FCCOB[7]=ReceDataBuff[3];
      FTFC->FCCOB[8]=ReceDataBuff[4];
      FTFC->FCCOB[9]=ReceDataBuff[5];
      FTFC->FCCOB[10]=ReceDataBuff[6];
      FTFC->FCCOB[11]=ReceDataBuff[7];
      FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*开始执行FLASH擦除命令,写1清除*/
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*等待命令完成*/
};
#pragma arm section
/*=================================================写入EEPROM数据(32位)===========================================================*/
void  WriteEepromData(unsigned int *Target,unsigned int  *Source,unsigned int size)
{
unsigned int cycles = size / 4;
unsigned int remain = size % 4;
unsigned int offset=0;
      DISABLE_INTERRUPTS();         /*关闭任务调度*/
	for (int i=0;i<cycles;i++)
	{
		while ((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK) == 0){};
		*((int*)(Target + offset))=*((int*)(Source + offset));
		offset ++;
	}
	for (int i=0;i<remain;i++)
	{
            while ((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK) == 0){};
            *((char*)Target + offset)=*((char*)Source + offset);
            offset++;
      };
      __NOP();
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*等待命令完成*/
      ENABLE_INTERRUPTS();          /*使能任务调度*/
};
/*=================================================写入EEPROM数据(16位)===========================================================*/
void  WriteEepromShort(unsigned short *Target,unsigned short  *Source,unsigned int size)
{
unsigned int offset=0;
      DISABLE_INTERRUPTS();         /*关闭任务调度*/
	for (int i=0;i<size;i++)
	{
            while ((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK) == 0){};
            *(Target + offset)=*(Source + offset);
            offset++;
      };
      __NOP();
      __NOP();
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*等待命令完成*/
      ENABLE_INTERRUPTS();          /*使能任务调度*/
};

/*==============================================设置FLEXRAM为EEPROM仿真模式=========================================*/
void  FlexramEeprom(void)
{
      if((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK)==0)   /*系统内部EEPROM没准备好*/
      {
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {};    /*等待上一次FLASH操作完成*/
            FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*清除相关报警*/
            FTFC->FCCOB[3]=EEPROM_SE_FlEXRAM;         /*设置FLEXRAM工作模式FCCOB0*/
            FTFC->FCCOB[2]=EEPROM_EMULATED;           /*FLEXRAM工作在仿真EEPROM模式,FCCOB1*/
            FTFC->FSTAT=FTFC_FSTAT_CCIF(1);           /*开始执行FLASH擦除命令,写1清除*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*等待命令完成*/
      }
};
/*==============================================初始化QSPI模块=====================================================
      功能说明:
            初始化低功耗SPI模块,设定SPI通讯的前端频率为4MHz,后端通讯频率又命令字分频决定.
      参数:
            Channel:定义使用的SPI模块通道,一共有三个SPI通道(SpiChannel0,SpiChannel0,SpiChannel0)
            DmaMode:定义SPI是否使用DMA通讯,1:启动DMA;0:不启动DMA;
===================================================================================================================*/
void  InitLPSpi(SPICHANNEL Channel,unsigned int DmaMode)
{
LPSPI_Type  *LPSpiPoint;

      switch(Channel)
      {
            case  SpiChannel0:
                  LPSpiPoint=LPSPI0;
                  PCC->PCCn[PCC_LPSPI0_INDEX] = 0;                /* Disable clocks to modify PCS ( default) */
                  PCC->PCCn[PCC_LPSPI0_INDEX] = 0xC6000000;       /* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) */
                  break;
            case  SpiChannel1:
                  LPSpiPoint=LPSPI1;
                  PCC->PCCn[PCC_LPSPI1_INDEX] = 0;                /* Disable clocks to modify PCS ( default) */
                  PCC->PCCn[PCC_LPSPI1_INDEX] = 0xC6000000;       /* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) */
                  break;
            case  SpiChannel2:
                  LPSpiPoint=LPSPI2;
                  PCC->PCCn[PCC_LPSPI2_INDEX] = 0;                /* Disable clocks to modify PCS ( default) */
                  PCC->PCCn[PCC_LPSPI2_INDEX] = 0xC6000000;       /* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) */
                  break;
      };
      LPSpiPoint->CR=0;       /*配置关闭形影的SPI模块*/
      LPSpiPoint->IER=0;      /*配置关闭SPI模块中断响应*/
      if(DmaMode == 1) LPSpiPoint->DER=0X03;   /*启动对应SPI模块的DMA传输功能*/
      else LPSpiPoint->DER=0;       /*禁止对应SPI模块的DMA传输功能*/
      LPSpiPoint->CFGR0=0;    /*接收数据匹配禁止、FIFO循环禁止、主机请求禁止、*/
      LPSpiPoint->CFGR1=0X04000001; /*外设片选使能、片选关闭后输出为三态、Sin输出输入、Sout为数据输出、输入匹配禁止、外设片选低有效、自动片选上拉禁止、SPI工作在主模式*/
//      LPSpiPoint->CCR=0X03030308;   /*SCKPCS=3+1、PCSSCK=3+1、DBT=3+2、SCKDIV=8+2（4MHz）*/
      LPSpiPoint->CCR=0X13130308;   /*SCKPCS=19+1、PCSSCK=19+1、DBT=3+2、SCKDIV=8+2（4MHz）*/
      LPSpiPoint->FCR=0X00000003;   /*RXWATER=0,Rx的FIFO>0产生RX标志；TXWATER=3，TX的FIFO<3产生TX标志*/
      LPSpiPoint->CR=0X00000009;    /*使能相应的LPSpi模块*/
};
/*========================================================InitATD初始化A/D转换=====================================================================*/
void InitATD(void)
{
      PCC->PCCn[PCC_ADC0_INDEX]=0;
//      PCC->PCCn[PCC_ADC1_INDEX]=0;
      PCC->PCCn[PCC_ADC0_INDEX]=PCC_PCCn_PCS(1) | PCC_PCCn_CGC(1);      /*使能ADC0gonzuo时钟,PCS-=1*/
//      PCC->PCCn[PCC_ADC1_INDEX]=PCC_PCCn_PCS(1) | PCC_PCCn_CGC(1);      /*使能ADC0gonzuo时钟,PCS-=1*/
      /*==================================ADC0模块设置=============================================*/
      ADC0->CFG1 = 0x000000004;     /* ADICLK=0: Input clk=ALTCLK1=SOSCDIV2 */
                                    /* ADIV=0: Prescaler=1 */
                                    /* MODE=1: 12-bit conversion */
      ADC0->CFG2 = 0x00000000C;     /* SMPLTS=12(default): sample time is 13 ADC clks */
      ADC0->SC2 = 0x00000000;       /* ADTRG=0: SW trigger */
                                    /* ACFE,ACFGT,ACREN=0: Compare func disabled */
                                    /* DMAEN=0: DMA disabled */
                                    /* REFSEL=0: Voltage reference pins= VREFH, VREEFL */
      ADC0->SC3 = 0x00000007;       /* CAL=0: Do not start calibration sequence */
                                    /* ADCO=0: One conversion performed */
                                    /* AVGE=1,AVGS=32: HW average function disabled */      
      /*==================================ADC1模块设置=============================================*/
//      ADC1->CFG1 = 0x000000004;     /* ADICLK=0: Input clk=ALTCLK1=SOSCDIV2 */
                                    /* ADIV=0: Prescaler=1 */
                                    /* MODE=1: 12-bit conversion */
//      ADC1->CFG2 = 0x00000000C;     /* SMPLTS=12(default): sample time is 13 ADC clks */
//      ADC1->SC2 = 0x00000000;       /* ADTRG=0: SW trigger */
                                    /* ACFE,ACFGT,ACREN=0: Compare func disabled */
                                    /* DMAEN=0: DMA disabled */
                                    /* REFSEL=0: Voltage reference pins= VREFH, VREEFL */
//      ADC1->SC3 = 0x00000007;       /* CAL=0: Do not start calibration sequence */
                                    /* ADCO=0: One conversion performed */
                                    /* AVGE=1,AVGS=32: HW average function disabled */      
};
/*=======================================================设置ADC自动校正功能===================================================================*/
void AutoCalibrationATD(void)
{
      /*========================自动校正ADC0===========================*/
      ADC0->CLPS = 0x00u;
      ADC0->CLP3 = 0x00u;
      ADC0->CLP2 = 0x00u;
      ADC0->CLP1 = 0x00u;
      ADC0->CLP0 = 0x00u;
      ADC0->CLPX = 0x00u;
      ADC0->CLP9 = 0x00u;
      ADC0->SC3 =ADC0->SC3 | 0X80;  /*启动自动校正*/
      while(ADC0->SC3 & 0X80){};
      /*========================自动校正ADC1===========================*/
//      ADC1->CLPS = 0x00u;
//      ADC1->CLP3 = 0x00u;
//      ADC1->CLP2 = 0x00u;
//      ADC1->CLP1 = 0x00u;
//      ADC1->CLP0 = 0x00u;
//      ADC1->CLPX = 0x00u;
//      ADC1->CLP9 = 0x00u;
//      ADC1->SC3 =ADC1->SC3 | 0X80;  /*启动自动校正*/
//      while(ADC1->SC3 & 0X80){};

};
/*=========================================================批量启动A/D转换======================================================================*/
void  StartAtdConver(void)
{
//      /*===============================ATD0通道切换====================================================*/
      ADC0->SC1[0]&=~ADC_SC1_ADCH_MASK;         /* Clear prior ADCH bits */
      ADC0->SC1[0] = ADC_SC1_ADCH(Atd0Pos);     /*启动A/D转换*/
//      /*================================ATD1通道切换====================================================*/
//      ADC1->SC1[0]&=~ADC_SC1_ADCH_MASK;         /* Clear prior ADCH bits */
//      ADC1->SC1[0] = ADC_SC1_ADCH(Atd1Pos);     /*启动A/D转换*/
};

/*======================================================获得A/D转换结果===============================================================*/
void  GetAtdResult(void)
{
unsigned  int     TempResult;
      /*=======================================获取ATD0模块A/D转换结果数值============================================*/
	TempResult=ADC0->R[0];			/*取出上一次A/D转换结果*/
      switch(Atd0Pos)
      {
            case  UB_VOLTAGE_CHANNEL:        			/*系统供电UB电压A/D转换通道*/
			UbVoltageBuff=UbVoltageBuff + TempResult - UbVoltageValue;
			UbVoltageValue=UbVoltageBuff / ATD_FILTER_SCALE;
			UbVoltage=UbVoltageValue * 900 / 1024;	/*UB电压值，精度0.01V*/
                  Atd0Pos=AD2428_VOUT2_CHANNEL;			/*AD2424输出电压Vout电压A/D转换通道*/
                  break;
            case  AD2428_VOUT2_CHANNEL:				/*AD2424输出电压Vout电压A/D转换通道*/
			Ad2428Vout2Buff=Ad2428Vout2Buff + TempResult - Ad2428Vout2Value;
			Ad2428Vout2Value=Ad2428Vout2Buff / ATD_FILTER_SCALE;
			Ad2428Vout2= Ad2428Vout2Value * 125 / 1024;	/*AD2428输出电压Vout2电压值，精度0.01V*/
                  Atd0Pos=AD2428_POWER_CHANNEL;			/*AD 2428供电电压A/D转换通道*/
                  break;
            case  AD2428_POWER_CHANNEL:          		/*AD 2428供电电压A/D转换通道*/
			Ad2428PowerBuff=Ad2428PowerBuff + TempResult - AD2428PowerValue;
			AD2428PowerValue=Ad2428PowerBuff / ATD_FILTER_SCALE;
			Ad2428Power=AD2428PowerValue 	* 125 / 512;		/*AD2428供电电压值，精度0.01V*/
                  Atd0Pos=UB_VOLTAGE_CHANNEL;        		/*系统供电UB电压A/D转换通道*/
                  break;
      };
};
/*====================================初始化PWM特征捕捉功能==============================================================*/
void  InitFtm2Capture(void)
{
      PCC->PCCn[PCC_FTM2_INDEX]=0;
      PCC->PCCn[PCC_FTM2_INDEX]=PCC_PCCn_PCS(1) | PCC_PCCn_CGC(1);      /*使能FTM1模块时钟,PCS-=1*/
      FTM2->MODE=FTM_MODE_WPDIS(1);       /*FTM1寄存器保护模式取消,可以输入数据; FTMEN=0*/
      FTM2->SC =0X00000003;         /* TOIE (Timer Overflow Interrupt Ena) = 0 (default) */
                                    /* CPWMS (Center aligned PWM Select) = 0 (default, up count) */
                                    /* CLKS (Clock source) = 0 (default, no clock; FTM disabled) */
                                    /* PS (Prescaler factor) = 3. Prescaler = 8 */
      FTM2->MOD=0;                  /*定时器模为0*/
      FTM2->CNTIN=0;                /*定时器初值为0*/
      FTM2->FILTER=FTM_FILTER_CH0FVAL(3);       /*2uS的滤波时间*/
      FTM2->COMBINE =0X0000000C;    /* FTM mode settings used: DECAP0=1, DECAPEN0=1, MCOMBINEx=0, COMBINEx=0 */    
      FTM2->SC =FTM2->SC | FTM_SC_CLKS(3);      /*定时器开始启动*/
}
/*===================================开始捕捉PWM输入特征=================================================================
输入参数:
      Mode:工作模式,0:捕捉PWM高电平时间;1:捕捉PWM周期时间,精度为1uS
=======================================================================================================================*/
void  StartFtm2Capture(unsigned int Mode)
{
      if(Mode)    /*捕捉周期长度*/
      {
            FTM2->CONTROLS[0].CnSC=0X00000014;        /*捕捉上升沿,连续模式*/      
            FTM2->CONTROLS[1].CnSC=0X00000004;        /*捕捉上升沿*/
      }else       /*捕捉高电平长度*/
      {
            FTM2->CONTROLS[0].CnSC=0X00000014;        /*捕捉上升沿,连续模式*/      
            FTM2->CONTROLS[1].CnSC=0X00000008;        /*捕捉下降沿*/
      };
      if(FTM2->CONTROLS[0].CnSC & FTM_CnSC_CHF_MASK) FTM2->CONTROLS[0].CnSC=FTM2->CONTROLS[0].CnSC & (~FTM_CnSC_CHF_MASK);  /*清除标志位*/
      if(FTM2->CONTROLS[1].CnSC & FTM_CnSC_CHF_MASK) FTM2->CONTROLS[1].CnSC=FTM2->CONTROLS[1].CnSC & (~FTM_CnSC_CHF_MASK);  /*清除标志位*/
};
/*======================================读PWM特征数据================================================================
输入参数:
      Mode:工作模式,0:捕捉PWM高电平时间;1:捕捉PWM周期时间,精度为1uS
=======================================================================================================================*/
void  ReadFtm2CaptureData(unsigned int Mode)
{
//unsigned short      TempC0V,TempC1V;
//      if((FTM2->STATUS & 0X03)==3)       /*成功发生双沿捕捉*/
//      {
//            TempC0V=(unsigned short)(FTM2->CONTROLS[0].CnV);
//            TempC1V=(unsigned short)((unsigned short)(FTM2->CONTROLS[1].CnV)-(unsigned short)TempC0V);
//            if(Mode)
//            {
//                  SysControlMessage.PwmCycleValue=TempC1V;
//            }else
//            {
//                  if( TempC1V < SysControlMessage.PwmCycleValue)
//                        SysControlMessage.PwmValue=(unsigned int)TempC1V * 1000 / SysControlMessage.PwmCycleValue;
 //                 else  SysControlMessage.PwmValue=0;
 //           };
 //     }else
 //     {
 //           SysControlMessage.PwmValue=0;
 //           SysControlMessage.PwmCycleValue=0;
 //     };
};
/*===================================================发送LPSPI数据==============================================================
输入参数： Channel：LPSPI通道号，取值范围SpiChannel0 ,SpiChannel1 ,SpiChannel2。
            Buff：要发送数据的缓冲区。
            Size：要发送数据的字节数。
            CsUnmber:片选号码，0-3表示：PCS0-PCS3。
            Cpol/Cpha：LPSPI发送数据的相位。
===============================================================================================================================*/
void  TransLPSpi(SPICHANNEL Channel,unsigned char *Buff,unsigned char Size,unsigned char CsNumber,unsigned char Cpol,unsigned char Cpha)
{
unsigned char     *TempPoint;
unsigned    int   *TempTcr;
unsigned char     TempSize,TempNumber;
      if((Size <= 16) && (Size > 0))   /*SPI一次最多传输16个字节,4个缓冲区*/
      {
            switch(Channel)
            {
                  case  SpiChannel0:
                        TempPoint=(unsigned char *)(&LPSPI0->TDR);
                        TempTcr=(unsigned int *)(&LPSPI0->TCR);
                        break;
                  case  SpiChannel1:
                        TempPoint=(unsigned char *)(&LPSPI1->TDR);
                        TempTcr=(unsigned int *)(&LPSPI1->TCR);
                        break;
                  case  SpiChannel2:
                        TempPoint=(unsigned char *)(&LPSPI2->TDR);
                        TempTcr=(unsigned int *)(&LPSPI2->TCR);
                        break;
            };
            TempNumber=0;
            while(TempNumber < Size)
            {
                  memcpy(TempPoint,Buff+TempNumber,4);        /*将数据传送到发送缓冲区*/
                  TempNumber=TempNumber+4;
            };
            TempSize=(Size * 8) - 1;            /*计算SPI发送数据的位数*/
            /*====设置SPI传送命令,CPOL;CPHA;PRESCALE=3;PCS;BYSW=1:字节位置交换;WIDTH=0====*/
            *TempTcr=(Cpol << 31) + (Cpha << 30) + 0X18400000 +  (CsNumber << 24) + TempSize;
      };
};
/*===================================================读LPSPI接收数据===========================================================
输入参数： Channel：LPSPI通道号，取值范围SpiChannel0 ,SpiChannel1 ,SpiChannel2。
            Buff：要读取数据的缓冲区。
            Size：要读取数据的字节数。
==============================================================================================================================*/
void  ReadLPSpi(SPICHANNEL Channel,unsigned char *Buff,unsigned char Size)
{
unsigned char     *TempPoint;
unsigned char     TempNumber;

            if((Size <= 16) && (Size > 0))   /*SPI一次最多传输16个字节,4个缓冲区*/
      {
            switch(Channel)
            {
                  case  SpiChannel0:
                        TempPoint=(unsigned char *)(&LPSPI0->RDR);
                        break;
                  case  SpiChannel1:
                        TempPoint=(unsigned char *)(&LPSPI1->RDR);
                        break;
                  case  SpiChannel2:
                        TempPoint=(unsigned char *)(&LPSPI2->RDR);
                        break;
            };
            TempNumber=0;
            while(TempNumber < Size)
            {
                  memcpy(Buff + TempNumber,TempPoint,4);
                  TempNumber=TempNumber+4;
            };
      }
};
/*==============================================清空LPSPI接收缓冲区(FIFO)========================================================
输入参数： Channel：LPSPI通道号，取值范围SpiChannel0 ,SpiChannel1 ,SpiChannel2。
  返回值:  接收缓冲区中的最后一个数据.
================================================================================================================================*/
unsigned int  ClearLPSpiReceFIFO(SPICHANNEL Channel)
{
LPSPI_Type  *LPSpiPoint;
unsigned int      TempValue;  /*临时变量*/
      switch(Channel)
      {
            case  SpiChannel0:
                  LPSpiPoint=LPSPI0;
                  break;
            case  SpiChannel1:
                  LPSpiPoint=LPSPI1;
                  break;
            case  SpiChannel2:
                  LPSpiPoint=LPSPI2;
                  break;
      };
      while((LPSpiPoint->RSR & 0X02) == 0)      /*接收缓冲区(FIFO)不空*/
      {
            TempValue=LPSpiPoint->RDR;
      };
      return TempValue;
};
/*===========================================通过DMA方式与外置EEPROM进行SPI通讯==========================================
功能说明: 
      该函数完成SPI通讯,使用DMA方式,一次传输批量数据通过SPI发送给EEPROM.
参数:
      Tran:SPI发送数据缓冲区地址.
      Rece:SPI接收数据缓冲区地址.
      Size:一次传送的数据数量(Byte个数)
=========================================================================================================================*/
void EepromSpiDma(unsigned char *Tran,unsigned char *Rece,int ByteSize)
{
int   TempCount;
int   TempSize;
      if(ByteSize>0)    /*SPI发送的数据必须大于一个字节*/
      {
            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
            else TempCount=ByteSize /4 ;
            /*================================设置DMA通道12的TCB配置(SPI1 RECE)================================*/
            DMA->TCD[10].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(TempCount);        /*在主循环内共计有1个小循环*/     
            DMA->TCD[10].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(TempCount);        /*在主循环内共计有1个小循环*/
            DMA->TCD[10].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(4) ;  
            DMA->TCD[10].SADDR=DMA_TCD_SADDR_SADDR(&LPSPI1->RDR);           /*SPI1通道DMA传输源地址(SPI1接收寄存器)*/ 
            DMA->TCD[10].SOFF=0;    /*源地址偏移量*/
            DMA->TCD[10].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,32位数据格式*/
            DMA->TCD[10].SLAST=0;    /*主循环完成后源地址回调*/
            DMA->TCD[10].DADDR=DMA_TCD_DADDR_DADDR(Rece);     /*SPI1通道DMA传输目标地址(SPI1接收缓冲区)*/
            DMA->TCD[10].DOFF=4;    /*目标地址偏移量*/
            DMA->TCD[10].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-4 * TempCount);     /*主循环完成后目标地址回调*/
            DMA->TCD[10].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(1)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            /*================================设置DMA通道11的TCB配置(SPI1 TRAN)================================*/
            DMA->TCD[9].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(TempCount);        /*在主循环内共计有1个小循环*/     
            DMA->TCD[9].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(TempCount);        /*在主循环内共计有1个小循环*/
            DMA->TCD[9].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(4) ;  
            DMA->TCD[9].SADDR=DMA_TCD_SADDR_SADDR(Tran);       /*SPI1通道DMA传输源地址(SPI1发送)*/ 
            DMA->TCD[9].SOFF=4;          /*源地址偏移量*/
            DMA->TCD[9].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,32位数据格式*/
            DMA->TCD[9].SLAST=DMA_TCD_DLASTSGA_DLASTSGA(-4 * TempCount);         /*主循环完成后源地址回调*/
            DMA->TCD[9].DADDR=DMA_TCD_DADDR_DADDR(&LPSPI1->TDR);       /*SPI1通道DMA传输目标地址(SPI1接收缓冲区)*/
            DMA->TCD[9].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[9].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[9].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(1)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            TempSize=(ByteSize * 8) - 1;              /*计算SPI发送数据的位数*/
            LPSPI1->TCR =0X18400000 + TempSize;       /*设置SPI0传送命令,CPOL=0;CPHA=0;PRESCALE=3;PCS=PCS0;BYSW=1:字节位置交换;WIDTH=0*/
            
            DMA->SERQ=DMA_SERQ_SERQ(10);        /*使能SPI1 RECE通道DMA服务*/
            DMA->SERQ=DMA_SERQ_SERQ(9);         /*使能SPI1 TRAN通道DMA服务*/
      };
};
/*=====================================================初始化FTM0的各项配置===============================================
功能说明: 
      该函数完成初始化FTM0的各项配置.
参数:
      无
返回值：
			无
=========================================================================================================================*/
void FTM0_init(void)
{
	/**
	 * FTM0 Clocking:
	 * ==================================================
	 */
	PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK; 	/* Ensure clk disabled for config 	*/
	//使能FTM0时钟
	PCC->PCCn[PCC_FTM0_INDEX] |= PCC_PCCn_PCS(1)	/* Clock Src=1, 8 MHz SIRCDIV1_CLK */
                              |  PCC_PCCn_CGC_MASK;  	/* Enable clock for FTM regs 		*/

	/*!
	 * FTM0 Initialization:
	 * ===================================================
	 */
	FTM0->MODE |= FTM_MODE_WPDIS_MASK;  /* Write protect to registers disabled (default) 				*/
	FTM0->SC	=		FTM_SC_PWMEN0_MASK	/* Enable PWM channel 0 output	*/
					|FTM_SC_PWMEN1_MASK	/* Enable PWM channel 1 output	*/
					|FTM_SC_PWMEN2_MASK	/* Enable PWM channel 2 output	*/
					|FTM_SC_PWMEN3_MASK	/* Enable PWM channel 3 output	*/
					|FTM_SC_PWMEN4_MASK	/* Enable PWM channel 4 output	*/
					|FTM_SC_PWMEN5_MASK	/* Enable PWM channel 5 output	*/
					|FTM_SC_PWMEN6_MASK	/* Enable PWM channel 6 output									*/
					|FTM_SC_PWMEN7_MASK 	/* Enable PWM channel 7 output									*/
					|FTM_SC_PS(0)
					|FTM_SC_TOIE(0);     	/* TOIE (Timer Overflow Interrupt Ena) = 0 (default) 			*/
										/* CPWMS (Center aligned PWM Select) = 0 (default, up count) 	*/
										/* CLKS (Clock source) = 0 (default, no clock; FTM disabled) 	*/
										/* PS (Prescaler factor) = 1. Prescaler = 1 					*/

	FTM0->COMBINE = 0x00000000;	/* FTM mode settings used: DECAPENx, MCOMBINEx, COMBINEx=0 */
	FTM0->POL = 0x00000000;    	/* Polarity for all channels is active high (default) 		*/
	FTM0->MOD = 1667 -1 ;     	/* FTM1 counter final value (used for PWM mode) 			*/
								/* FTM1 Period = MOD-CNTIN+0x0001 ~= 62500 ctr clks  		*/
								/* 8MHz /1 = 8000kHz ->  ticks -> 48kHz 					*/
//	FTM0_CH6_PWM_init(70);			
//	FTM0_CH7_PWM_init(30);			
}
/*=====================================================初始化FTM0的各项配置===============================================
功能说明: 
      该函数完成初始化FTM1的各项配置.
参数:
      无
返回值：
			无
=========================================================================================================================*/
void FTM1_init(void)
{
	/**
	 * FTM0 Clocking:
	 * ==================================================
	 */
	PCC->PCCn[PCC_FTM1_INDEX] &= ~PCC_PCCn_CGC_MASK; 	/* Ensure clk disabled for config 	*/
	//使能FTM0时钟
	PCC->PCCn[PCC_FTM1_INDEX] |= PCC_PCCn_PCS(1)	/* Clock Src=1, 8 MHz SIRCDIV1_CLK */
                              |  PCC_PCCn_CGC_MASK;  	/* Enable clock for FTM regs 		*/

	/*!
	 * FTM0 Initialization:
	 * ===================================================
	 */
	FTM1->MODE |= FTM_MODE_WPDIS_MASK;  /* Write protect to registers disabled (default) 				*/
	FTM1->SC	=		FTM_SC_PWMEN0_MASK	/* Enable PWM channel 0 output	*/
					|FTM_SC_PWMEN1_MASK	/* Enable PWM channel 1 output	*/
					|FTM_SC_PWMEN2_MASK	/* Enable PWM channel 2 output	*/
					|FTM_SC_PWMEN3_MASK	/* Enable PWM channel 3 output	*/
					|FTM_SC_PWMEN4_MASK	/* Enable PWM channel 4 output	*/
					|FTM_SC_PWMEN5_MASK	/* Enable PWM channel 5 output	*/
					|FTM_SC_PWMEN6_MASK	/* Enable PWM channel 6 output	*/
					|FTM_SC_PWMEN7_MASK 	/* Enable PWM channel 7 output	*/
					|FTM_SC_PS(0)
					|FTM_SC_TOIE(0);     	/* TOIE (Timer Overflow Interrupt Ena) = 0 (default) 			*/
										/* CPWMS (Center aligned PWM Select) = 0 (default, up count) 	*/
										/* CLKS (Clock source) = 0 (default, no clock; FTM disabled) 	*/
										/* PS (Prescaler factor) = 1. Prescaler = 1 					*/

	FTM1->COMBINE = 0x00000000;	/* FTM mode settings used: DECAPENx, MCOMBINEx, COMBINEx=0 */
	FTM1->POL = 0x00000000;    	/* Polarity for all channels is active high (default) 		*/
	FTM1->MOD = 1667 -1 ;     	/* FTM1 counter final value (used for PWM mode) 			*/
								/* FTM1 Period = MOD-CNTIN+0x0001 ~= 62500 ctr clks  		*/
								/* 8MHz /1 = 8000kHz ->  ticks -> 48kHz 					*/
//	FTM1_CH6_PWM_init(70);			
//	FTM1_CH7_PWM_init(30);			
}
/*======================================================================FTM模块开始工作===================================================
Module：模块号，0，1，2
=======================================================================================================================================*/
void StartFtmCounter (unsigned int	Module)
{
	switch(Module)
	{
		case 	0:
			FTM0->SC |= FTM_SC_CLKS(1);
			break;
		case	1:
			FTM1->SC |= FTM_SC_CLKS(1);
			break;
		case 	2:
			FTM2->SC |= FTM_SC_CLKS(1);
			break;
	};
//	FTM0->SC |= FTM_SC_CLKS(3);
	/* Start FTM0 counter with clk source = external clock (SOSCDIV1_CLK)*/
}

/*===============================================================初始化FTM0 PWM配置======================================================
功能说明: 
      该函数完成初始化FTM0的各项配置,主要为占空比.
参数:
	Channel:通道号：0，1，2，3，4，5，6，7
      duty:占空比（0-100）
返回值：
			无
=======================================================================================================================================*/
void FTM0_PWM_init(unsigned int Channel,unsigned int duty)
{
	switch(Channel)
	{
		case	0:
			FTM0->CONTROLS[0].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[0].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	1:
			FTM0->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[1].CnV =  (FTM0->MOD+1)*duty/100;;			/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	2:
			FTM0->CONTROLS[2].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[2].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	3:
			FTM0->CONTROLS[3].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[3].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	4:
			FTM0->CONTROLS[4].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[4].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	5:
			FTM0->CONTROLS[5].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses ；EN_DMA	*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[5].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	6:
			FTM0->CONTROLS[6].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[6].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	7:
			FTM0->CONTROLS[7].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK|0X41;  /* FTM0 ch6: edge-aligned PWM, low true pulses En_DMA*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM0->CONTROLS[7].CnV =  (FTM0->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
	};
}

/*===============================================================初始化FTM0 PWM配置======================================================
功能说明: 
      该函数完成初始化FTM1的各项配置,主要为占空比.
参数:
	Channel:通道号：0，1，2，3，4，5，6，7
      duty:占空比（0-100）
返回值：
			无
=======================================================================================================================================*/
void FTM1_PWM_init(unsigned int Channel,unsigned int duty)
{
	/**
	 * FTM0, Channel 6 in PWM Mode:
	 * ==================================================
	 */
	switch(Channel)
	{
		case	0:
			FTM1->CONTROLS[0].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[0].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	1:
			FTM1->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK|0X41;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 	，Enable DMA*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[1].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	2:
			FTM1->CONTROLS[2].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[2].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	3:
			FTM1->CONTROLS[3].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[3].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	4:
			FTM1->CONTROLS[4].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[4].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	5:
			FTM1->CONTROLS[5].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK |0X41;  	/* FTM0 ch6: edge-aligned PWM, low true pulses ，Enable DMA	*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[5].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	6:
			FTM1->CONTROLS[6].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 		*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[6].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
		case	7:
			FTM1->CONTROLS[7].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK|0X41 ;  	/* FTM0 ch6: edge-aligned PWM, low true pulses ，Enable DMA	*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[7].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
	};
}
/*=============================================================FMT0，DMA0数据传输==========================================================================*/
void Fmt0Dma0Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT0，DMA0数据传输*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI发送的数据必须大于一个字节*/
      {

            /*================================设置DMA通道0的TCB配置================================*/
            DMA->TCD[0].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有DataSize个小循环*/     
            DMA->TCD[0].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有DataSize个小循环*/
            DMA->TCD[0].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA传输一次、2个字节数据*/
            DMA->TCD[0].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[0].SOFF=2;          /*源地址偏移量*/
            DMA->TCD[0].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[0].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*主循环完成后源地址回调*/
            DMA->TCD[0].DADDR=DMA_TCD_DADDR_DADDR(&(FTM0->CONTROLS[7].CnV));       /*SPI0通道DMA传输目标地址(FTM0-CH7接收缓冲区)*/
            DMA->TCD[0].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[0].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[0].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
//            DMA->TCD[0].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(1)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(0);        
		
      };
};
/*========================================================FMT1，DMA1数据传输============================================================================*/
void Fmt1Dma1Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT1，DMA1数据传输*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI发送的数据必须大于一个字节*/
      {
//            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
//            TempCount=DataSize ;

            /*================================设置DMA通道0的TCB配置================================*/
            DMA->TCD[1].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有DataSize个小循环*/     
            DMA->TCD[1].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有DataSize个小循环*/
            DMA->TCD[1].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA传输一次、2个字节数据*/
            DMA->TCD[1].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[1].SOFF=2;          /*源地址偏移量*/
            DMA->TCD[1].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[1].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*主循环完成后源地址回调*/
            DMA->TCD[1].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[1].CnV));       /*SPI0通道DMA传输目标地址(FTM0-CH5接收缓冲区)*/
            DMA->TCD[1].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[1].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[1].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(1);        
		
      };
};
/*========================================================FMT1，DMA2数据传输============================================================================*/
void Fmt1Dma2Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT1，DMA1数据传输*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI发送的数据必须大于一个字节*/
      {
//            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
//            TempCount=DataSize ;

            /*================================设置DMA通道0的TCB配置================================*/
            DMA->TCD[2].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有DataSize个小循环*/     
            DMA->TCD[2].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有DataSize个小循环*/
            DMA->TCD[2].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA传输一次、2个字节数据*/
            DMA->TCD[2].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[2].SOFF=2;          /*源地址偏移量*/
            DMA->TCD[2].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[2].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*主循环完成后源地址回调*/
            DMA->TCD[2].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[7].CnV));       /*SPI0通道DMA传输目标地址(FTM0-CH5接收缓冲区)*/
            DMA->TCD[2].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[2].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[2].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(2);        
		
      };
};
/*========================================================FMT1，DMA3数据传输============================================================================*/
void Fmt1Dma3Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT1，DMA1数据传输*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI发送的数据必须大于一个字节*/
      {
//            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
//            TempCount=DataSize ;

            /*================================设置DMA通道0的TCB配置================================*/
            DMA->TCD[3].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有DataSize个小循环*/     
            DMA->TCD[3].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有DataSize个小循环*/
            DMA->TCD[3].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA传输一次、2个字节数据*/
            DMA->TCD[3].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[3].SOFF=2;          /*源地址偏移量*/
            DMA->TCD[3].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[3].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*主循环完成后源地址回调*/
            DMA->TCD[3].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[5].CnV));       /*SPI0通道DMA传输目标地址(FTM0-CH5接收缓冲区)*/
            DMA->TCD[3].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[3].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[3].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(3);        
		
      };
};
/*===================================================================司机座椅声音播放==============================================================================
LeftSound：左声道数据缓冲区
RightSound：右声道数据缓冲区
DataSize：声音缓冲区长度
==================================================================================================================================================================*/
void	DriverMusicOutput(unsigned char *LeftSound,unsigned	char *RightSound,unsigned int DataSize)	
{
	if(DataSize>0)    /*SPI发送的数据必须大于一个字节*/
      {
		/*=============================================左声道DMA设置=========================================*/
            DMA->TCD[0].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有DataSize个小循环*/     
            DMA->TCD[0].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有DataSize个小循环*/
            DMA->TCD[0].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA传输一次、2个字节数据*/
            DMA->TCD[0].SADDR=DMA_TCD_SADDR_SADDR(LeftSound);       
            DMA->TCD[0].SOFF=2;          /*源地址偏移量*/
            DMA->TCD[0].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[0].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*主循环完成后源地址回调*/
            DMA->TCD[0].DADDR=DMA_TCD_DADDR_DADDR(&(FTM0->CONTROLS[7].CnV));       /*SPI0通道DMA传输目标地址(FTM0-CH7接收缓冲区)*/
            DMA->TCD[0].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[0].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[0].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
		/*=============================================右声道DMA设置=========================================*/
            DMA->TCD[1].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有DataSize个小循环*/     
            DMA->TCD[1].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有DataSize个小循环*/
            DMA->TCD[1].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA传输一次、2个字节数据*/
            DMA->TCD[1].SADDR=DMA_TCD_SADDR_SADDR(RightSound);       
            DMA->TCD[1].SOFF=2;          /*源地址偏移量*/
            DMA->TCD[1].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[1].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*主循环完成后源地址回调*/
            DMA->TCD[1].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[1].CnV));       /*SPI0通道DMA传输目标地址(FTM0-CH5接收缓冲区)*/
            DMA->TCD[1].DOFF=0;          /*目标地址偏移量*/
            DMA->TCD[1].DLASTSGA=0;      /*主循环完成后目标地址回调*/
            DMA->TCD[1].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
		/*================================================使能DMA=========================================*/
		DMA->SERQ=DMA_SERQ_SERQ(0);        /*使能SPI0 TRAN通道DMA服务*/
		DMA->SERQ=DMA_SERQ_SERQ(1);        /*使能SPI0 TRAN通道DMA服务*/
	};
};

/*=============================================================初始化FLEX_IO -->IIS========================================================
IIS工作条件（进输入）：
	FLEXIO_D3：IIS数据输入PIN。
	FLEXIO_D4：IIS数据同步（SYNC。）PIN，左右声道识别
	FLEXIO_D5：IIS数据移位时钟输入PIN。
==========================================================================================================================================*/
void	FlexIOInit(void)
{
	/*===================================使能FLEX_IO时钟=========================================*/	
	PCC->PCCn[PCC_FlexIO_INDEX] = 0;                /* Disable clocks to modify PCS ( default) */
	PCC->PCCn[PCC_FlexIO_INDEX] = 0xC6000000;       /* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) */
/*==============================================================FLEX_IO配置1===========================================================*/		
//	FLEXIO->SHIFTCFG[0] = FLEXIO_SHIFTCFG_SSTART(0)				/*Start位无效，接收器在第一个移位加载数据*/
//                               | FLEXIO_SHIFTCFG_SSTOP(0)
//                               | FLEXIO_SHIFTCFG_INSRC(0);			/*PIN*/
//	FLEXIO->SHIFTCTL[0] = FLEXIO_SHIFTCTL_SMOD(1)				/*Receive接收模式*/
//                                | FLEXIO_SHIFTCTL_PINPOL(0)			/*不适用与接收模式*/
//					  | FLEXIO_SHIFTCTL_PINSEL(3)			/*FLEX_IO:D3，数据采集端口*/
//                                | FLEXIO_SHIFTCTL_PINCFG(0)			/*不适用与接收模式*/
//                                | FLEXIO_SHIFTCTL_TIMPOL(1)			/*下降沿数据移位*/
//                                | FLEXIO_SHIFTCTL_TIMSEL(0);		/*选择0号定时器配合移位*/
	/*==================================================Configure SCK timer =========================================================*/
//	FLEXIO->TIMCMP[0] = 63;								/*（FLEXIO->TIMCMP[0]+1）/2：总的数据位数，H/L：左右声道*/
//	FLEXIO->TIMCFG[0] = FLEXIO_TIMCFG_TSTART(0)				/*定时器没有起始位*/
//                            | FLEXIO_TIMCFG_TSTOP(0)				/*定时器没有停止位*/
//                            | FLEXIO_TIMCFG_TIMENA(0)				/*定时器时钟有效*/
//                            | FLEXIO_TIMCFG_TIMDIS(0)				/*定时器时钟有效*/
//                            | FLEXIO_TIMCFG_TIMRST(6)				/*定时器在外部触发上升沿复位*/
//                           | FLEXIO_TIMCFG_TIMDEC(2)				/*定时器递减计数来自PIN（双沿），移位寄存器等效*/
//                            | FLEXIO_TIMCFG_TIMOUT(0);			/*定时器无输出*/
//	FLEXIO->TIMCTL[0] = FLEXIO_TIMCTL_TIMOD(3)				/*16位定时器模式*/
//			          | FLEXIO_TIMCTL_PINSEL(5)				/*定时器递减信号输入，FLEX_IO:D5*/
//                            | FLEXIO_TIMCTL_PINPOL(0)				/*接收模式不关注*/
//                            | FLEXIO_TIMCTL_PINCFG(0)				/*定时器没有输出*/
//                            | FLEXIO_TIMCTL_TRGSRC(1)				/*定时器内部触发*/
//                            | FLEXIO_TIMCTL_TRGPOL(0)				/*定时器无触发输出*/
//                            | FLEXIO_TIMCTL_TRGSEL(8);			/*定时器触发有PIN针FLEX_IO:D4*/
/*==============================================================FLEX_IO配置2(OK)===========================================================*/	
	FLEXIO->SHIFTCFG[1] = 0X0;
	FLEXIO->SHIFTCTL[1] = 0X01030602;		/*D6输出IIS,Timer1,上升沿移位输出*/
	
	FLEXIO->SHIFTCFG[0] = 0X0;
	FLEXIO->SHIFTCTL[0] = 0X00800301;		/*D3输入IIS,timer0，下降沿移位输入*/
//	FLEXIO->SHIFTCTL[0] = 0X00100301;		/*D3输入IIS,timer0，上升沿移位输入*/

	/* Configure SCK timer */
	FLEXIO->TIMCMP[0] = 63;								/*（FLEXIO->TIMCMP[0]+1）/2：总的数据位数，H/L：左右声道*/
	FLEXIO->TIMCFG[0] = 0X00202500;
	FLEXIO->TIMCTL[0] = 0X0B400583;		/*IIS时钟信号D5,定时器2输出触发*/
	
	FLEXIO->TIMCMP[1] = 63;								/*（FLEXIO->TIMCMP[0]+1）/2：总的数据位数，H/L：左右声道*/
	FLEXIO->TIMCFG[1] = 0X00202500;
	FLEXIO->TIMCTL[1] = 0X03400583;		/*IIS时钟信号D5*/

	FLEXIO->TIMCMP[2] = 0;								/*（FLEXIO->TIMCMP[0]+1）/2：总的数据位数，H/L：左右声道*/
	FLEXIO->TIMCFG[2] = 0X00206400;
	FLEXIO->TIMCTL[2] = 0x0A400483;		/*IIS时钟信号D5*/
/*==============================================================FLEX_IO配置3===========================================================*/		
//	FLEXIO->SHIFTCFG[0] = FLEXIO_SHIFTCFG_SSTART(0)				/*Start位无效，接收器在第一个移位加载数据*/
//					| FLEXIO_SHIFTCFG_SSTOP(0)
//					| FLEXIO_SHIFTCFG_INSRC(0);			/*PIN*/
//	FLEXIO->SHIFTCTL[0] = FLEXIO_SHIFTCTL_SMOD(1)				/*Receive接收模式*/
//					| FLEXIO_SHIFTCTL_PINPOL(0)			/*不适用与接收模式*/
//					| FLEXIO_SHIFTCTL_PINSEL(3)			/*FLEX_IO:D3，数据采集端口*/
//					| FLEXIO_SHIFTCTL_PINCFG(0)			/*接收模式,输出禁止*/
//					| FLEXIO_SHIFTCTL_TIMPOL(1)			/*下降沿数据移位*/
//					| FLEXIO_SHIFTCTL_TIMSEL(0);		/*选择0号定时器配合移位*/
//	/*==================================================Configure SCK timer =========================================================*/
//	FLEXIO->TIMCMP[0] = 33;								/*（FLEXIO->TIMCMP[0]+1）/2：总的数据位数，H/L：左右声道*/
//	FLEXIO->TIMCFG[0] = FLEXIO_TIMCFG_TSTART(0)				/*定时器没有起始位*/
//					| FLEXIO_TIMCFG_TSTOP(0)			/*定时器没有停止位*/
//					| FLEXIO_TIMCFG_TIMENA(6)			/*定时器使能并移位条件：触发信号上升沿D4：SYNC*/
//					| FLEXIO_TIMCFG_TIMENA(5)			/*定时器使能并移位条件：触发信号高电平D4：SYNC；移位信号上升沿*/
//					| FLEXIO_TIMCFG_TIMDIS(2)			/*定时器禁止条件：定时器比较完成后*/
//					| FLEXIO_TIMCFG_TIMRST(0)			/*定时器复位条件：永远不复位*/
//					| FLEXIO_TIMCFG_TIMDEC(2)			/*定时器递减计数来自PIN：D3（双沿），移位寄存器等效*/
//					| FLEXIO_TIMCFG_TIMOUT(0);			/*定时器初始化输出1：不受定时器复位影响*/
//	FLEXIO->TIMCTL[0] = FLEXIO_TIMCTL_TIMOD(3)				/*16位定时器模式*/
//					| FLEXIO_TIMCTL_PINPOL(0)			/*接收模式不关注*/
//					| FLEXIO_TIMCTL_PINSEL(5)			/*定时器递减信号输入，FLEX_IO:D5*/
//					| FLEXIO_TIMCTL_PINCFG(0)			/*定时器为输入模式，输出禁止*/
//					| FLEXIO_TIMCTL_TRGSRC(1)			/*定时器内部触发*/
//					| FLEXIO_TIMCTL_TRGPOL(0)			/*定时器无触发输出*/
//					| FLEXIO_TIMCTL_TRGSEL(8);			/*定时器触发有PIN针FLEX_IO:D4*/
/*=================================================================使能DMA=============================================================*/
	FLEXIO->SHIFTSIEN=0X01;				/*移位寄存器0 中断使能*/
	FLEXIO->SHIFTSDEN=0X01;				/*移位寄存器0 DMA使能*/
	FLEXIO->CTRL=1;					/*使能FLEX_IO模块使能*/
};

/*====================================================================================================================================*/
void FlexioDmaRece(unsigned char *ReceBuff,int DataSize)				/*Flex_IO，DMA12数据传输*/
{
      if(DataSize>0)    /*SPI发送的数据必须大于一个字节*/
      {

            /*================================设置DMA通道0的TCB配置================================*/
            DMA->TCD[12].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*在主循环内共计有1个小循环*/     
            DMA->TCD[12].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*在主循环内共计有1个小循环*/
            DMA->TCD[12].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(4) ;  		 /*DMA传输一次、4个字节数据*/
            DMA->TCD[12].SADDR=DMA_TCD_SADDR_SADDR(&FLEXIO->SHIFTBUFBIS[0]);       
            DMA->TCD[12].SOFF=0;          /*源地址偏移量*/
            DMA->TCD[12].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*源/目的地址模操作,16位数据格式*/
            DMA->TCD[12].SLAST=0;         /*主循环完成后源地址回调*/
            DMA->TCD[12].DADDR=DMA_TCD_DADDR_DADDR(ReceBuff);       /**/
            DMA->TCD[12].DOFF=4;          /*目标地址偏移量*/
            DMA->TCD[12].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-4*DataSize);     /*主循环完成后目标地址回调*/
            DMA->TCD[12].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
		DMA->SERQ=DMA_SERQ_SERQ(12);        /*使能FLEX_IO通道DMA服务*/
		
      };
};
/*==========================================================================================================================*/
unsigned	int	GetIisDmaPosition(unsigned int TotalBufferSize)		/*返回DMA接收IIS数据在缓冲区中的位置*/
{
	return TotalBufferSize-DMA->TCD[12].CITER.ELINKNO;
};
/*==========================================================================================================================*/
void	DisableDma(unsigned int Channel)				/*关闭相应的DMA通道*/
{
	if(Channel <= 15)
		DMA->CERQ=DMA_CERQ_CERQ(Channel);
};
/*==========================================================================================================================*/
unsigned	int	GetSoundDrvPosition(unsigned int TotalBufferSize)		/*返回DMA声音驱动数据在缓冲区中的位置*/	
{
	return TotalBufferSize-DMA->TCD[0].CITER.ELINKNO;
};
	



