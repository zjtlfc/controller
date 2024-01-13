#include "S32K144_API.h"
#include "system_S32K144.h"             // Keil::Device:Startup
#include "clock_S32K1xx.h"              // Keil::Device:SDK Driver:Clock
#include "clock_manager.h"              // Keil::Device:SDK Driver:Clock
#include "string.h"
//#include "S32K144.h"
#include "function.h"
#include "s32_core_cm4.h"
/*===========================================================���ⳣ��������===========================================================*/
#if	RESET_VECTOR_HANDLE == 1		/*�ֶ�ǿ�ƶ�λ��λ������ַ*/
extern	void Reset_Handler(void);
const	unsigned int	PushPollPointer __attribute__((at(0x0000))) = 0X20007000;					/*�����ϵ��ʼ����ջָ��*/
const	unsigned int	ResetVectorPROC __attribute__((at(0x0004))) =(unsigned int)Reset_Handler;		/*�����ϵ��ʼ���׵�ַ*/
#endif
/*===============================================================ȫ�ֱ���������=======================================================*/
struct CAN_MESSAGE_BUFFER   Can0TranBuffer[CAN0_TRAN_BUFF_SIZE];              /*CAN0���ͻ���ȥ*/
struct CAN_MESSAGE_BUFFER   Can1TranBuffer[CAN1_TRAN_BUFF_SIZE];              /*CAN1���ͻ���ȥ*/
struct CAN_MESSAGE_BUFFER   Can2TranBuffer[CAN2_TRAN_BUFF_SIZE];              /*CAN2���ͻ���ȥ*/
struct CAN_MESSAGE_BUFFER   Can0ReceBuffer[CAN0_RECE_BUFF_SIZE];              /*CAN0���ջ�����*/
struct CAN_MESSAGE_BUFFER   Can1ReceBuffer[CAN1_RECE_BUFF_SIZE];              /*CAN1���ջ�����*/
struct CAN_MESSAGE_BUFFER   Can2ReceBuffer[CAN2_RECE_BUFF_SIZE];              /*CAN2���ջ�����*/
int   Can0TranTop,Can0TranTail;                                               /*CAN0���ͻ�����ͷβָ��*/
int   Can1TranTop,Can1TranTail;                                               /*CAN1���ͻ�����ͷβָ��*/
int   Can2TranTop,Can2TranTail;                                               /*CAN2���ͻ�����ͷβָ��*/
int   Can0RecePosition,Can1RecePosition,Can2RecePosition;                     /*Can0��Can1��Can2���ջ�����ָ��λ��*/
unsigned int	HostCommandState;								/*������������״̬*/
unsigned int	SystemTicksPrev;								/*ϵͳʱ����Ļ���*/
unsigned	char	HostData[8];								/*��λ�����͸�����*/	
unsigned	char	CcpDataBuff[8];								/*CCP�궨���ݻ�����*/
unsigned	int	Atd0Pos;									/*��ǰA/Dת��ͨ����*/
unsigned	int	UbVoltageValue,Ad2428Vout2Value,AD2428PowerValue;		/*��·��ѹ���A/Dת��ֵ*/
int	UbVoltageBuff,Ad2428Vout2Buff,Ad2428PowerBuff;					/*A/Dת���˲�������*/
unsigned	int	UbVoltage;									/*UB��ѹֵ������0.01V*/
unsigned	int	Ad2428Vout2;								/*AD2428�����ѹVout2��ѹֵ������0.01V*/
unsigned	int	Ad2428Power;								/*AD2428�����ѹֵ������0.01V*/
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

	SCG->SOSCCSR = SCG_SOSCCSR_SOSCERR(1U);		/*ϵͳʱ�Ӽ��ʹ��(�ⲿ��������)*/

	/* set up OSC clock. */
	/* Setup dividers. */
	SCG->SOSCDIV = (SCG_SOSCDIV_SOSCDIV1(1U) |
                  SCG_SOSCDIV_SOSCDIV2(1U)  );	/*ϵͳOSCʱ�ӱ�������*/

	/* Set OSC configuration. */
	SCG->SOSCCFG = (SCG_SOSCCFG_EREFS(1U) |
                  SCG_SOSCCFG_RANGE(2U)  );	/*ʹ��ʯӢ����������Ƶ��Ϊ�е��ٶ�*/

	/* Enable clock. */
	SCG->SOSCCSR = (SCG_SOSCCSR_SOSCEN(1U) );	/*ʹ��ϵͳʯӢ����*/

	/* Wait for OSC clock to be valid. */
	while (!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD(1U))) __NOP();	/*�ȴ����ʯӢ���������ȶ�*/

	/* Init SIRC */
	SCG->SIRCCSR = 0U;	/*����Slow IRC��ع��ܹر�*/

	/* Setup dividers. */
	SCG->SIRCDIV = (SCG_SIRCDIV_SIRCDIV1(1U) |
                  SCG_SIRCDIV_SIRCDIV2(2U)  );	/*Slow IRC��������*/

	/* Set SIRC configuration. */
	SCG->SIRCCFG = SCG_SIRCCFG_RANGE(1U);	/*����Slow IRCƵ��Ϊ8MHz*/

	/* Enable clock. */
	SCG->SIRCCSR = SCG_SIRCCSR_SIRCEN(1U);	/*ʹ��Slow IRCʱ��*/

	/* Wait for SIRC clock to be valid. */
	while (!(SCG->SIRCCSR & SCG_SIRCCSR_SIRCVLD(1U))) __NOP();	/*�ȴ�Slow IRC�����ȶ�*/

	/* Change to SIRC as system clock source */
	SCG->RCCR = (SCG_RCCR_DIVSLOW(3U) |
               SCG_RCCR_DIVBUS(0U)  |
               SCG_RCCR_DIVCORE(0U) |
               SCG_RCCR_SCS(2U)     );	/*����ϵͳʱ��ΪSlow IRC������ʱ�ӱ�������Ϊ1����ʱ�ӱ�������Ϊ1��Slow Clock��������Ϊ3*/

	/* Wait for clock source switch finished. */
	while ((SCG->RCCR & 0x0F000000) != 0x02000000) __NOP();;	/*�ȴ�ʱ���л����*/

	/* Init Firc */
	SCG->FIRCCSR = 0U;	/*����Fast IRC��ع��ܹر�*/

	/* Setup dividers. */
	SCG->FIRCDIV = (SCG_FIRCDIV_FIRCDIV1(1U) |
                  SCG_FIRCDIV_FIRCDIV2(1U)  );	/*Fast IRC��������*/

	/* Set FIRC configuration. */
	SCG->FIRCCFG = SCG_FIRCCFG_RANGE(0U);	/*����Fast IRCƵ��Ϊ48MHz*/

	/* Enable clock. */
	SCG->FIRCCSR |= SCG_FIRCCSR_FIRCEN(1U);	/*ʹ��Fast IRC*/

	/* Wait for FIRC clock to be valid. */
	while (!(SCG->FIRCCSR & SCG_FIRCCSR_FIRCVLD(1U))) __NOP();;	/*�ȴ�Fast IRC�����ȶ�*/

	/* Init SysPll */
	SCG->SPLLCSR = SCG_SPLLCSR_SPLLERR(1U);		/*ϵͳ���໷���ʹ��*/

	/* Setup dividers. */
	SCG->SPLLDIV = (SCG_SPLLDIV_SPLLDIV1(1U) |
                  SCG_SPLLDIV_SPLLDIV2(2U)  );	/*�������໷��������*/

	/* Set PLL configuration. */
	SCG->SPLLCFG = (SCG_SPLLCFG_PREDIV(0U) |                     /* prediv 1 */
                  SCG_SPLLCFG_MULT(4U)   );                    /* pll multiply 20 */

	/* Enable clock. */
	SCG->SPLLCSR = SCG_SPLLCSR_SPLLEN(1U);	/*ʹ�����໷ʱ��*/

	/* Wait for PLL clock to be valid. */
	while (!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD(1U))) __NOP();	/*�ȴ����໷ʱ�ӹ����ȶ�*/

	/* Change to use PLL as system clock source */
	SCG->RCCR = (SCG_RCCR_DIVSLOW(2U) |
               SCG_RCCR_DIVBUS(1U)  |
               SCG_RCCR_DIVCORE(0U) |
               SCG_RCCR_SCS(6U)     );	/*ϵͳʹ�����໷���ʱ�ӡ����߱�������Ϊ2���˱�������Ϊ1��Slow Clock��������Ϊ3*/

	/* Wait for clock source switch finished. */
	while ((SCG->RCCR & 0x0F000000) != 0x06000000) __NOP();	/*�ȴ�ʱ���л��ȶ�*/
}
/*=================================================��ʼ��ϵͳӲ��====================================================*/
void	InitHardware(void)
{
	/*=================================ʹ����������ʱ��============================================*/
	PCC->PCCn[PCC_PORTA_INDEX]=PCC_PCCn_CGC(1);	/*�˿�Aʱ��ʹ��*/
	PCC->PCCn[PCC_PORTB_INDEX]=PCC_PCCn_CGC(1);	/*�˿�Bʱ��ʹ��*/
	PCC->PCCn[PCC_PORTC_INDEX]=PCC_PCCn_CGC(1);	/*�˿�Cʱ��ʹ��*/
	PCC->PCCn[PCC_PORTD_INDEX]=PCC_PCCn_CGC(1);	/*�˿�Dʱ��ʹ��*/
	PCC->PCCn[PCC_PORTE_INDEX]=PCC_PCCn_CGC(1);	/*�˿�Eʱ��ʹ��*/
//      SIM->PLATCGC=SIM->PLATCGC | SIM_PLATCGC_CGCDMA(1);             /*DMAͨ��ʱ��Ĭ��Ϊ����*/
      PCC->PCCn[PCC_DMAMUX_INDEX] = PCC_PCCn_CGC(1);  /*ʹ��DMAMUXʱ��*/
      PCC->PCCn[PCC_FTFC_INDEX]=PCC_PCCn_CGC(1);	/*FTFC-FLASHʱ��ʹ��*/
	
	/*=============================���ö˿ڹ���====================================================*/
	PORTA->PCR[5]=PORT_PCR_MUX(7);			/*PTA5Ϊ��λ�����*/
	PORTA->PCR[17]=PORT_PCR_MUX(1);			/*PTA17Ϊһ��I/O�����*/
	PORTA->PCR[16]=PORT_PCR_MUX(1);			/*PTA16Ϊһ��I/O�����*/
	PORTA->PCR[15]=PORT_PCR_MUX(1);			/*PTA15Ϊһ��I/O�����*/
	PORTA->PCR[14]=PORT_PCR_MUX(1);			/*PTA14Ϊһ��I/O�����*/
	PORTA->PCR[13]=PORT_PCR_MUX(1);			/*PTA13Ϊһ��I/O�����*/
	PORTA->PCR[12]=PORT_PCR_MUX(1);			/*PTA12Ϊһ��I/O�����*/
	PORTA->PCR[11]=PORT_PCR_MUX(1);			/*PTA11Ϊһ��I/O�����*/
	PORTA->PCR[3]=PORT_PCR_MUX(1);			/*PTA3Ϊһ��I/O�����*/
	PORTA->PCR[2]=PORT_PCR_MUX(1);			/*PTA2Ϊһ��I/O�����*/
	
	PORTB->PCR[17]=PORT_PCR_MUX(1);			/*PTB17Ϊһ��I/O�����*/
	PORTB->PCR[16]=PORT_PCR_MUX(1);			/*PTB16Ϊһ��I/O�����*/
	PORTB->PCR[15]=PORT_PCR_MUX(1);			/*PTB15Ϊһ��I/O�����*/
	PORTB->PCR[14]=PORT_PCR_MUX(1);			/*PTB14Ϊһ��I/O�����*/
	PORTB->PCR[13]=PORT_PCR_MUX(1);			/*PTB13Ϊһ��I/O�����*/
	PORTB->PCR[12]=PORT_PCR_MUX(1);			/*PTB12Ϊһ��I/O�����*/
	PORTB->PCR[11]=PORT_PCR_MUX(1);			/*PTB11Ϊһ��I/O�����*/
	PORTB->PCR[8]=PORT_PCR_MUX(1);			/*PTB8Ϊһ��I/O�����*/
	PORTB->PCR[5]=PORT_PCR_MUX(2);			/*PTB5ΪFTM0_CH5���*/
	PORTB->PCR[4]=PORT_PCR_MUX(2);			/*PTB4ΪFTM0_CH4���*/
	PORTB->PCR[3]=PORT_PCR_MUX(2);			/*PTB3ΪFTM1_CH1���*/
	PORTB->PCR[2]=PORT_PCR_MUX(2);			/*PTB2ΪFTM1_CH0���*/
	PORTB->PCR[1]=PORT_PCR_MUX(1);			/*PTB1Ϊһ��I/O�����*/
	PORTB->PCR[0]=PORT_PCR_MUX(1);			/*PTB0Ϊһ��I/O�����*/
	
	PORTC->PCR[17]=PORT_PCR_MUX(1);			/*PTC17Ϊһ��I/O�����*/
	PORTC->PCR[16]=PORT_PCR_MUX(1);			/*PTC16Ϊһ��I/O�����*/
	PORTC->PCR[15]=PORT_PCR_MUX(2);			/*PTC15FTM1_CH3���*/
	PORTC->PCR[14]=PORT_PCR_MUX(2);			/*PTC14FTM1_CH2���*/
	PORTC->PCR[10]=PORT_PCR_MUX(1);			/*PTC10Ϊһ��I/O�����*/
	PORTC->PCR[9]=PORT_PCR_MUX(1);			/*PTC9Ϊһ��I/O�����*/
	PORTC->PCR[8]=PORT_PCR_MUX(1);			/*PTC8Ϊһ��I/O�����*/
	PORTC->PCR[3]=PORT_PCR_MUX(2);			/*PTC3ΪFTM0_CH3���*/
	PORTC->PCR[2]=PORT_PCR_MUX(2);			/*PTC2ΪFTM0_CH2���*/
	PORTC->PCR[1]=PORT_PCR_MUX(6);			/*PTC1ΪFTM1_CH7���*/
	PORTC->PCR[0]=PORT_PCR_MUX(6);			/*PTC0ΪFTM1_CH6���*/
	
	PORTD->PCR[17]=PORT_PCR_MUX(1);			/*PTD17Ϊһ��I/O�����*/
	PORTD->PCR[16]=PORT_PCR_MUX(2);			/*PTD16ΪFTM0_CH1���*/
	PORTD->PCR[15]=PORT_PCR_MUX(2);			/*PTD15ΪFTM0_CH0���*/
	PORTD->PCR[14]=PORT_PCR_MUX(1);			/*PTD14Ϊһ��I/O�����*/
	PORTD->PCR[13]=PORT_PCR_MUX(1);			/*PTD13Ϊһ��I/O�����*/
	PORTD->PCR[12]=PORT_PCR_MUX(1);			/*PTD12Ϊһ��I/O�����*/
	PORTD->PCR[11]=PORT_PCR_MUX(1);			/*PTD11Ϊһ��I/O�����*/
	PORTD->PCR[10]=PORT_PCR_MUX(1);			/*PTD10Ϊһ��I/O�����*/
	PORTD->PCR[9]=PORT_PCR_MUX(6);			/*PTD9ΪFTM1_CH5���*/
	PORTD->PCR[8]=PORT_PCR_MUX(6);			/*PTD8ΪFTM1_CH4���*/
	PORTD->PCR[7]=PORT_PCR_MUX(1);			/*PTD7Ϊһ��I/O�����*/
	PORTD->PCR[6]=PORT_PCR_MUX(1);			/*PTD6Ϊһ��I/O�����*/
	PORTD->PCR[5]=PORT_PCR_MUX(1);			/*PTD5Ϊһ��I/O�����*/
	PORTD->PCR[4]=PORT_PCR_MUX(1);			/*PTD4Ϊһ��I/O�����*/
	PORTD->PCR[3]=PORT_PCR_MUX(1);			/*PTD3Ϊһ��I/O�����*/
	PORTD->PCR[2]=PORT_PCR_MUX(1);			/*PTD2Ϊһ��I/O�����*/
	PORTD->PCR[1]=PORT_PCR_MUX(1);			/*PTD1Ϊһ��I/O�����*/
	PORTD->PCR[0]=PORT_PCR_MUX(1);			/*PTD0Ϊһ��I/O�����*/
//	PORTD->PCR[1]=PORT_PCR_MUX(6);			/*PTD1ΪFXIO_D1���*/
//	PORTD->PCR[0]=PORT_PCR_MUX(6);			/*PTD0ΪFXIO_D0���*/
	
	PORTE->PCR[16]=PORT_PCR_MUX(6);			/*PTE16ΪFXIO_D3���*/
	PORTE->PCR[15]=PORT_PCR_MUX(6);			/*PTE15ΪFXIO_D2���*/
	PORTE->PCR[14]=PORT_PCR_MUX(1);			/*PTE14Ϊһ��I/O�����*/
	PORTE->PCR[11]=PORT_PCR_MUX(6);			/*PTE11ΪFXIO_D5���*/
	PORTE->PCR[10]=PORT_PCR_MUX(6);			/*PTE10ΪFXIO_D4���*/
	PORTE->PCR[9]=PORT_PCR_MUX(2);			/*PTE9ΪFXIO_D7���*/
	PORTE->PCR[8]=PORT_PCR_MUX(2);			/*PTE8ΪFXIO_D6���*/
	PORTE->PCR[6]=PORT_PCR_MUX(1);			/*PTE6Ϊһ��I/O�����*/
	PORTE->PCR[2]=PORT_PCR_MUX(1);			/*PTE2Ϊһ��I/O�����*/
	PORTE->PCR[1]=PORT_PCR_MUX(1);			/*PTE1Ϊһ��I/O�����*/
	PORTE->PCR[0]=PORT_PCR_MUX(1);			/*PTE0Ϊһ��I/O�����*/
	/*==========================IIC���߶˿�����========================================*/
//	PORTA->PCR[2]=PORT_PCR_MUX(3);			/*PTA2ΪIIC_SDA���*/
//	PORTA->PCR[3]=PORT_PCR_MUX(3);			/*PTA3ΪIIC_SCL���*/
	/*==========================CAN���߶˿�����========================================*/
	PORTE->PCR[4]=PORT_PCR_MUX(5);			/*����PTD4ΪCAN0_RXͨѶ��*/
	PORTE->PCR[5]=PORT_PCR_MUX(5);			/*����PTD5ΪCAN0_TXͨѶ��*/
	
//	PORTA->PCR[12]=PORT_PCR_MUX(3);			/*����PTA12ΪCAN1_RXͨѶ��*/
//	PORTA->PCR[13]=PORT_PCR_MUX(3);			/*����PTA13ΪCAN1_TXͨѶ��*/
	
//	PORTC->PCR[16]=PORT_PCR_MUX(3);			/*����PTC16λCAN2_RXͨѶ��*/
//	PORTC->PCR[17]=PORT_PCR_MUX(3);			/*����PTC17λCAN2_TXͨѶ��*/
      /*==========================LPSPI�˿�����==========================================*/
//      PORTB->PCR[4]=PORT_PCR_MUX(3);                  /*LPSPIģ��SPI0ͨ��(SOUT)*/
//      PORTB->PCR[5]=PORT_PCR_MUX(4);                  /*LPSPIģ��SPI0ͨ��(PCS0)*/
//      PORTD->PCR[15]=PORT_PCR_MUX(4);                 /*LPSPIģ��SPI0ͨ��(SCK)*/
//      PORTD->PCR[16]=PORT_PCR_MUX(4);                 /*LPSPIģ��SPI0ͨ��(SIN)*/
      
//      PORTD->PCR[1]=PORT_PCR_MUX(3);                  /*LPSPIģ��SPI1ͨ��(SIN)*/
//      PORTD->PCR[0]=PORT_PCR_MUX(3);                  /*LPSPIģ��SPI1ͨ��(PCS0)*/
//      PORTE->PCR[1]=PORT_PCR_MUX(5);                  /*LPSPIģ��SPI1ͨ��(SCK)*/
//      PORTB->PCR[16]=PORT_PCR_MUX(3);                 /*LPSPIģ��SPI1ͨ��(SOUT)*/

//      PORTE->PCR[11]=PORT_PCR_MUX(2);                 /*LPSPIģ��SPI2ͨ��(PCS0)*/
//      PORTE->PCR[15]=PORT_PCR_MUX(3);                 /*LPSPIģ��SPI2ͨ��(SCK)*/
//      PORTE->PCR[16]=PORT_PCR_MUX(3);                 /*LPSPIģ��SPI2ͨ��(SIN)*/
//      PORTA->PCR[8]=PORT_PCR_MUX(3);                  /*LPSPIģ��SPI2ͨ��(SOUT)*/
//      PORTE->PCR[10]=PORT_PCR_MUX(3);                 /*LPSPIģ��SPI2ͨ��(PCS1)*/
      /*============================A/Dת��ģ��˿�����====================================*/
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
      /*=================================���벶׽�˿�����======================================*/
//      PORTD->PCR[10]=PORT_PCR_MUX(2);                  /*FTM2_CH0���벶׽�˿�*/
	/*==================================�˿ڷ����趨===========================================*/
	PTA->PDDR=0X3F80C;					/*PTA17��16��15��14��13��12��11��3��2Ϊ�����*/
	PTB->PDDR=0X003F;						/*PTB0��1��2��3��4��5Ϊ����˿�*/
	PTC->PDDR=0XC70F;						/*PTC10��9��8��14��15��0��1��2��3Ϊ����˿�*/
	PTD->PDDR=0X3FF7F;					/*PTD17��PTD8��PTD6��PTD0Ϊ�����*/
	PTE->PDDR=0X4347;						/*PTE0��1��2��6��8��9��14Ϊ�����*/
      
	DISABLE_SCL_CLEAR;					/*PTA3=0*/
	DISABLE_SDA_CLEAR;					/*PTA2=0*/
	DISABLE_PTD_PD0;						/*PTD0=0*/
	DISABLE_PTD_PD1;						/*PTD1=0*/
	
	POWER_LOCK_ENABLE;					/*����ʹ�ܵ�Դ����,PTC10=1*/

      /*================================InitATD��ʼ��A/Dת��===================================*/
	Atd0Pos=UB_VOLTAGE_CHANNEL;		/*��ǰA/Dת��ͨ����:ϵͳ����UB��ѹA/Dת��ͨ��*/
	UbVoltageValue=0;				/*UB��ѹADת��ֵ*/
	Ad2428Vout2Value=0;			/*AD2428�����ѹVout��A/Dת��ֵ*/
	AD2428PowerValue=0;			/*AD2428�����ѹA/Dת��ֵ*/
	UbVoltageBuff=0;				/*A/Dת���˲�������*/
	Ad2428Vout2Buff=0;			/*A/Dת���˲�������*/
	Ad2428PowerBuff=0;			/*A/Dת���˲�������*/
      InitATD();
      AutoCalibrationATD();                           /*ADC�Զ�У��*/    
      WATCHDOG_TRIGGER;       /*�������Ź�ιʳ,PTC9��ת*/ 
      /*===============================��ʼ��PWM������׽����=====================================*/
	FTM0_init();				/*��ʼ��FTM0ģ��*/	
	FTM1_init();				/*��ʼ��FTM1ģ��*/	
//      InitFtm2Capture();                                             
      /*===================================��ʼ��DMAģ��=======================================*/
      DmaInitial();
	/*==================================��ʼ��CAN����ģ��=====================================*/
	CAN_EN_ENABLE;			/*ʹ��CAN_EN,PTE1=1*/
	CAN_STB_ENABLE;			/*ʹ��CAN_STB,PTE0=1*/	
	InitCan(CanChannel0,CAN_500);				/*CAN0��ʼ��������500Khz��CAN0Ϊ����CANͨ��*/
      /*===================================��ʼ��LPSpiģ��======================================*/
//      InitLPSpi(SpiChannel0,1);                       /*��ʼ��QSPI0ģ�飬ʹ��DMA����*/
//      InitLPSpi(SpiChannel1,1);                       /*��ʼ��QSPI1ģ�飬ʹ��DMA����*/
//      InitLPSpi(SpiChannel2,0);                       /*��ʼ��QSPI2ģ�飬��ֹDMA����*/
      /*======================================FLEXRAM��������====================================*/
      WATCHDOG_TRIGGER;       /*�������Ź�ιʳ,PTC9��ת*/ 
      FlexRamParden();                                        
      /*=================================����FLEXRAMΪEEPROM����ģʽ=============================*/
      WATCHDOG_TRIGGER;       /*�������Ź�ιʳ,PTC9��ת*/ 
      FlexramEeprom();              
      /*====================================�ж��������¶�λ====================================*/
      S32_SCB->VTOR=0X8000;                           /*�ж����������¶�λ��0X8000��,�׶�FLASH����Bootloader��*/
}
/*============================================ʹ��CAN����ʱ��====================================
	����ֵ:NULL
	����:Channel��һ��ö�ٱ�������ʾCAN���ߵ�ͨ����
================================================================================================*/	
void 	EnableCanClock(CANCHANNEL Channel)
{
	switch(Channel)
	{
		case	CanChannel0:
			PCC->PCCn[PCC_FlexCAN0_INDEX]=PCC_PCCn_CGC(1);	/*ʹ��CAN0ʱ��Դ*/
		break;
		case	CanChannel1:
			PCC->PCCn[PCC_FlexCAN1_INDEX]=PCC_PCCn_CGC(1);	/*ʹ��CAN1ʱ��Դ*/
		break;
		case	CanChannel2:
			PCC->PCCn[PCC_FlexCAN2_INDEX]=PCC_PCCn_CGC(1);	/*ʹ��CAN2ʱ��Դ*/
		break;
	};
};
/*=====================================ʹCANģ����븴λ״̬=====================================
	����ֵ:NULL
	����:Channel��һ��ö�ٱ�������ʾCAN���ߵ�ͨ����
================================================================================================*/
void 	ResetCanModule(CANCHANNEL Channel)
{
	switch(Channel)
	{
		case	CanChannel0:
			CAN0->MCR=CAN0->MCR | CAN_MCR_SOFTRST(1);		/*CAN0����CAN���߸�λģʽ*/
			while(CAN0->MCR & CAN_MCR_SOFTRST(1)) {};		/*�ȴ�CAN���߸�λ���*/
		break;
		case	CanChannel1:
			CAN1->MCR=CAN1->MCR | CAN_MCR_SOFTRST(1);		/*CAN0����CAN���߸�λģʽ*/
			while(CAN1->MCR & CAN_MCR_SOFTRST(1)) {};		/*�ȴ�CAN���߸�λ���*/
		break;
		case	CanChannel2:
			CAN2->MCR=CAN2->MCR | CAN_MCR_SOFTRST(1);		/*CAN0����CAN���߸�λģʽ*/
			while(CAN2->MCR & CAN_MCR_SOFTRST(1)) {};		/*�ȴ�CAN���߸�λ���*/
		break;
	};
};
/*=====================================================��ʼ��CAN����==========================================
	����ֵ����
	��  ����
		Channel��CANģ��ţ�����һ��CANCHANNEL��������ʾCAN������ģ��{CanChannel0 ,CanChannel1 ,CanChannel2}
		baud��CANͨѶ���ʣ�����һ��CANBAUD������{CAN_1000,CAN_500,CAN_250,CAN_200,CAN_125,CAN_100,CAN_50}
=============================================================================================================*/
void 	InitCan(CANCHANNEL Channel,CANBAUD baud)				/*CAN0��ʼ������*/
{
CAN_Type *CanPoint;
unsigned char	PreDivide;		/*ʱ��Ԥ��Ƶ��*/	
unsigned char	ReceMnSize;		/*����λ��,������������,��������λ��*/
unsigned char	TranMnPos;	
unsigned char	MnPos;
	switch(Channel)
	{
		case	CanChannel0:
			CanPoint=(CAN_Type *)CAN0_BASE;			/*������Ӧ��CANָ���׵�ַ*/
			ReceMnSize=CAN0_RECE_MN_SIZE;				/*CAN0�����������䳤��*/
			TranMnPos=CAN0_TRAN_MN_POS;				/*CAN0������������λ��*/
		break;
		case	CanChannel1:
			CanPoint=(CAN_Type *)CAN1_BASE;			/*������Ӧ��CANָ���׵�ַ*/
			ReceMnSize=CAN1_RECE_MN_SIZE;				/*CAN1�����������䳤��*/
			TranMnPos=CAN1_TRAN_MN_POS;				/*CAN1������������λ��*/
		break;
		case	CanChannel2:
			CanPoint=(CAN_Type *)CAN2_BASE;			/*������Ӧ��CANָ���׵�ַ*/
			ReceMnSize=CAN2_RECE_MN_SIZE;				/*CAN2�����������䳤��*/
			TranMnPos=CAN2_TRAN_MN_POS;				/*CAN2������������λ��*/
		break;
	};
	switch(baud)
	{
		case	CAN_1000:
			PreDivide=0;	/*Ԥ��Ƶ����Ϊ1*/
		break;
		case	CAN_500:
			PreDivide=1;	/*Ԥ��Ƶ����Ϊ2*/
		break;
		case	CAN_250:
			PreDivide=3;	/*Ԥ��Ƶ����Ϊ4*/
		break;
		case	CAN_200:
			PreDivide=4;	/*Ԥ��Ƶ����Ϊ5*/
		break;
		case	CAN_125:
			PreDivide=7;	/*Ԥ��Ƶ����Ϊ8*/
		break;
		case	CAN_100:
			PreDivide=9;	/*Ԥ��Ƶ����Ϊ10*/
		break;
		case	CAN_50:
			PreDivide=19;	/*Ԥ��Ƶ����Ϊ20*/
		break;
		default:
			PreDivide=1;	/*Ĭ��Ԥ��Ƶ����Ϊ2*/
		break;
	};
      /*==================================ʹ��CAN0����ʱ��========================================*/
	EnableCanClock(Channel);				            
	/*============================================================����MCR==============================================================*/
      /*=================================Disable CANģ��===========================================*/
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
	/*================================CAN����CAN���߸�λģʽ====================================*/  
		CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_SOFTRST_MASK) | CAN_MCR_SOFTRST(1U);		
		while (((CanPoint->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) != 0U) {}
		while ((CanPoint->MCR & CAN_MCR_SOFTRST_MASK)==0) {};      /*�ȴ���λ���*/
      /*===================================ʹ��CANģ��=============================================*/      
      if(((CanPoint->MCR & CAN_MCR_MDIS_MASK) >> CAN_MCR_MDIS_SHIFT))
      {
            CanPoint->MCR=CanPoint->MCR & 0X7FFFFFFF;				
            while (((CanPoint->MCR & CAN_MCR_LPMACK_MASK) >> CAN_MCR_LPMACK_SHIFT)) {};     
      };           
      /*=================================CANģ�����Freezeģʽ====================================*/
      CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(1U);
      if((CanPoint->MCR & CAN_MCR_HALT_MASK)==0)
      {
            CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_HALT_MASK) | CAN_MCR_HALT(1U);
            while (((CanPoint->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) == 0U) {}
      };
      
      /*================================���ջ�����=================================================*/
      CanPoint->RXMGMASK=0;
      CanPoint->RX14MASK=0;
      
      CanPoint->MCR=CanPoint->MCR | CAN_MCR_SRXDIS(1);            /*�رշ���,������ѭ��*/

      CanPoint->RX15MASK=0;      
      CanPoint->IMASK1=0;                                         /*�ر�MB�ж�*/
	/*============================================�����Ϣ������===============================================*/
	MnPos=0;								/*��ʼֵΪ0*/
	while(MnPos<(ReceMnSize + 1)*4)		/*��ʼ����������*/
	{
		CanPoint->RAMn[MnPos]=0X0;					      /*������״̬*/
		MnPos++;
	};
	/*=========================================================================================================*/
      CanPoint->MCR=CanPoint->MCR | CAN_MCR_RFEN(1);              /*CAN���߽���FIFOģʽ*/
      CanPoint->RXFGMASK=0;                                       /*FIFOȫ�����μĴ���*/
      CanPoint->MCR=CanPoint->MCR | CAN_MCR_DMA(1);               /*CAN���߽���DMAģʽ*/
      
	CanPoint->MCR=(CanPoint->MCR & 0XFFFFFF00)+ (ReceMnSize+1);	/*MB��������*/
	/*===============����CTRL1===============*/
	CanPoint->CTRL1=CAN_CTRL1_PRESDIV(PreDivide) | CAN_CTRL1_PSEG2(2) | CAN_CTRL1_PSEG1(2) | CAN_CTRL1_LBUF(1);
	/*===============����CBT==================*/
	CanPoint->CBT=CAN_CBT_EPSEG2(2) | CAN_CBT_EPSEG1(2)| CAN_CBT_EPRESDIV(PreDivide);
	/*================================CANģ���뿪Freezeģʽ================================*/
      CanPoint->MCR = (CanPoint->MCR & ~CAN_MCR_FRZ_MASK) | CAN_MCR_FRZ(0U);
      while (((CanPoint->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT) != 0U) {}      
	/*================================����CAN���߷���/��������=============================*/
      /*============================================���ý�������===============================================*/
//	MnPos=0;								/*��ʼֵΪ0*/
//	while(MnPos<ReceMnSize)		/*��ʼ����������*/
//	{
//		CanPoint->RAMn[MnPos*4]=0X0;					      /*������״̬*/
//            CanPoint->RAMn[MnPos*4+1]=0X1FFFFFFF;
//		CanPoint->RAMn[MnPos*4]=0X04000000;					/*����Ϊ���ܻ���������Ծ״̬*/
//            CanPoint->RXIMR[MnPos]=0;                                   /*��������Ĵ���*/
//		MnPos++;
//	};
//      CanPoint->RAMn[0]=0X04000000;	
//      CanPoint->RXIMR[0]=0;                                   /*��������Ĵ���*/
//      PreDivide=CanPoint->TIMER;             /*�����������*/
//	MnPos=1;								/*��ʼֵΪ0*/
//	while(MnPos<ReceMnSize)		/*��ʼ����������*/
//	{
//		CanPoint->RAMn[MnPos*4]=0X0;					      /*������״̬*/
//            CanPoint->RAMn[MnPos*4+1]=0XFFFFFFFF;
//		CanPoint->RAMn[MnPos*4]=0X04000000;					/*����Ϊ���ܻ���������Ծ״̬*/
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
//      /*=========================================���÷�������λ��===============================================*/	
	CanPoint->RAMn[TranMnPos*4]=0X08;					/*����Ϊ���ͻ�����������Ծ״̬*/
};
/*====================================================����CAN����================================================
	����ֵ:
            0�����ͳɹ���
            -1�����ͻ�����æ���޷����б��η��͡�
	����:
		Channel��CANģ��ţ�����һ��CANCHANNEL��������ʾCAN������ģ��{CanChannel0 ,CanChannel1 ,CanChannel2}
		CanData:CAN���Ļ��������ݽṹ.
================================================================================================================*/
int  	TranCan(CANCHANNEL Channel,struct  	CAN_MESSAGE_BUFFER *CanData)
{
unsigned int	TransPos;		/*CAN��������λ��*/
unsigned int	TransCode;		/*���������������*/	
CAN_Type *CanPoint;
	switch(Channel)
	{
		case	CanChannel0:
			CanPoint=(CAN_Type *)CAN0_BASE;			/*������Ӧ��CANָ���׵�ַ*/
			TransPos=CAN0_TRAN_MN_POS;				/*CAN0������������λ��*/
		break;
		case	CanChannel1:
			CanPoint=(CAN_Type *)CAN1_BASE;			/*������Ӧ��CANָ���׵�ַ*/
			TransPos=CAN1_TRAN_MN_POS;				/*CAN1������������λ��*/
		break;
		case	CanChannel2:
			CanPoint=(CAN_Type *)CAN2_BASE;			/*������Ӧ��CANָ���׵�ַ*/
			TransPos=CAN2_TRAN_MN_POS;				/*CAN2������������λ��*/
		break;
	};
	TransCode=((CanPoint->RAMn[TransPos * 4]>> 24) & 0X0F );	/*�ó���ǰ������������*/
	if((TransCode==CAN_MB_TX_INACTIVE) || (TransCode==CAN_MB_RX_INACTIVE))	/*�������䴦�ڲ���Ծ״̬,���Է�������*/
	{
		CanData->CODE=CAN_MB_TX_DATA;		/*�������䴦�ڷ���״̬,��������������һ��*/
            CanPoint->RAMn[TransPos*4+2]=CAN_RAMn_DATA_BYTE_3(CanData->Data[3])+CAN_RAMn_DATA_BYTE_2(CanData->Data[2])+CAN_RAMn_DATA_BYTE_1(CanData->Data[1])+CAN_RAMn_DATA_BYTE_0(CanData->Data[0]);
            CanPoint->RAMn[TransPos*4+3]=CAN_RAMn_DATA_BYTE_3(CanData->Data[7])+CAN_RAMn_DATA_BYTE_2(CanData->Data[6])+CAN_RAMn_DATA_BYTE_1(CanData->Data[5])+CAN_RAMn_DATA_BYTE_0(CanData->Data[4]);
		(void)memcpy((unsigned char *)&(CanPoint->RAMn[TransPos * 4]),(unsigned char *)CanData,8);
            return 0;
	}else
      {     
            return -1;
      }
};
/*==================================================��ȡCAN���Ļ�������������=======================================
	����ֵ��
            0��CAN���ջ�������û�����ݡ�
            1��CAN���ջ�����������Ч��CAN���ݡ�
	������
		Channel��CANģ��ţ�����һ��CANCHANNEL��������ʾCAN������ģ��{CanChannel0 ,CanChannel1 ,CanChannel2}
		CanData:CAN���Ļ��������ݽṹ��
==================================================================================================================*/
int	ReadCanData(CANCHANNEL Channel,struct  CAN_MESSAGE_BUFFER *CanData)
{
struct CAN_MESSAGE_BUFFER *CanPoint;
int	ReceMbSize;		      /*CAN�������䳤��*/ 
int	MbPosition;	            /*DMA����CAN������λ��*/
int   *TempReadPosition;      /*��ǰ�������ճ���λ��*/
	switch(Channel)
	{
		case	CanChannel0:
			CanPoint=Can0ReceBuffer;			/*������Ӧ�Ľ��ջ������׵�ַ*/
			ReceMbSize=CAN0_RECE_BUFF_SIZE;	      /*CAN0�ջ��峤��*/
                  MbPosition=CAN0_RECE_BUFF_SIZE - (DMA->TCD[15].CITER.ELINKNO);      /*DMAͨ����ǰд����ջ�����λ��*/
                  TempReadPosition=&Can0RecePosition;
		break;
		case	CanChannel1:
			CanPoint=Can1ReceBuffer;			/*������Ӧ�Ľ��ջ������׵�ַ*/
			ReceMbSize=CAN1_RECE_BUFF_SIZE;	      /*CAN1�ջ��峤��*/
                  MbPosition=CAN1_RECE_BUFF_SIZE - (DMA->TCD[14].CITER.ELINKNO);      /*DMAͨ����ǰд����ջ�����λ��*/
                  TempReadPosition=&Can1RecePosition;
		break;
		case	CanChannel2:
			CanPoint=Can2ReceBuffer;			/*������Ӧ�Ľ��ջ������׵�ַ*/
			ReceMbSize=CAN2_RECE_BUFF_SIZE;	      /*CAN2�ջ��峤��*/
                  MbPosition=CAN2_RECE_BUFF_SIZE - (DMA->TCD[13].CITER.ELINKNO);      /*DMAͨ����ǰд����ջ�����λ��*/
                  TempReadPosition=&Can2RecePosition;
		break;
	};
	if(*TempReadPosition != MbPosition)		/*���ջ��������յ��µ�����*/
	{
                  memcpy(CanData , (unsigned char *)(&CanPoint[*TempReadPosition]) , 8);   /*ȡ����������ID��Ϣ*/
                  CanData->Data[0]=CanPoint[*TempReadPosition].Data[3];
                  CanData->Data[1]=CanPoint[*TempReadPosition].Data[2];
                  CanData->Data[2]=CanPoint[*TempReadPosition].Data[1];
                  CanData->Data[3]=CanPoint[*TempReadPosition].Data[0];
                  CanData->Data[4]=CanPoint[*TempReadPosition].Data[7];
                  CanData->Data[5]=CanPoint[*TempReadPosition].Data[6];
                  CanData->Data[6]=CanPoint[*TempReadPosition].Data[5];
                  CanData->Data[7]=CanPoint[*TempReadPosition].Data[4];
                  (*TempReadPosition)++;     /*ָ���ƶ�һλ*/
                  if(*TempReadPosition >= ReceMbSize) *TempReadPosition=0;  /*���ܻ�����Ϊѭ������*/   
                  return 1;
	}else       /*���ջ�����û���յ��µ�����*/
      {
            return 0;
      };
};
/*=====================================================���з���CAN����========================================================
	����ֵ:
            0�����ͳɹ���
            -1�����ͻ�����æ���޷����б��η��͡�
	����:
		Channel��CANģ��ţ�����һ��CANCHANNEL��������ʾCAN������ģ��{CanChannel0 ,CanChannel1 ,CanChannel2}
		CanData:CAN���Ļ��������ݽṹ.
============================================================================================================================*/
int      CANTranQueue(CANCHANNEL Channel,struct  CAN_MESSAGE_BUFFER *CanData)
{
int  *TempTopPoint,*TempTailPoint;
int   TempSize,TempValue;
struct CAN_MESSAGE_BUFFER     *TempBuff;
      DISABLE_INTERRUPTS();         /*�ر��������*/
	switch(Channel)
	{
		case	CanChannel0:                        /*CAN0���ͻ�����ͷβָ��*/
			TempTopPoint=&Can0TranTop;
			TempTailPoint=&Can0TranTail;
                  TempSize=CAN0_TRAN_BUFF_SIZE;       /*CAN0���ͻ���������*/
                  TempBuff=Can0TranBuffer;
		break;
		case	CanChannel1:                        /*CAN1���ͻ�����ͷβָ��*/
			TempTopPoint=&Can1TranTop;
			TempTailPoint=&Can1TranTail;
                  TempSize=CAN1_TRAN_BUFF_SIZE;       /*CAN1���ͻ���������*/
                  TempBuff=Can1TranBuffer;
		break;
		case	CanChannel2:                        /*CAN2���ͻ�����ͷβָ��*/
			TempTopPoint=&Can2TranTop;
			TempTailPoint=&Can2TranTail;
                  TempSize=CAN2_TRAN_BUFF_SIZE;       /*CAN2���ͻ���������*/
                  TempBuff=Can2TranBuffer;
		break;
	};
      TempValue=*TempTailPoint + 1;
      if(TempValue>=TempSize) TempValue=0;      /*ѭ������*/
      if(TempValue != *TempTopPoint)      /*���ͻ���������Ч��λ��*/
      {
            *TempTailPoint=TempValue;     /*�޸ķ��Ͷ���βָ��*/
            memcpy((unsigned char *)&TempBuff[TempValue],(unsigned char *)CanData,16);    /*��Ҫ���͵����ݿ��������ͻ�������*/
            ENABLE_INTERRUPTS();          /*ʹ���������*/
            return 0;
      }else
      {
            ENABLE_INTERRUPTS();          /*ʹ���������*/
            return -1;  /*���ͻ������Ѿ���,��������ʧ��*/
      }
      
};
/*======================================================DMA��ʼ��ģ��======================================================*/
void DmaInitial(void)
{
      /*======================================����DMAMUXͨ��,��ʹ��=======================================*/
      DMAMUX->CHCFG[15]=0;          /*DMAͨ��15ΪCAN0����FIFO,ʹ��DMAͨ��15*/
	
	
      DMAMUX->CHCFG[12]=0;          /*DMAͨ��12ΪFLEX_IO����,ʹ��DMAͨ��12*/
	
	
      DMAMUX->CHCFG[0]=0;          /*DMAͨ��0Ϊ��ʻԱ�����������ݽ���,ʹ��DMAͨ��0*/
      DMAMUX->CHCFG[1]=0;          /*DMAͨ��1Ϊ��ʻԱ�����������ݽ���,ʹ��DMAͨ��1*/

      DMAMUX->CHCFG[2]=0;          /*DMAͨ��2Ϊǰ��Ա�����������ݽ���,ʹ��DMAͨ��2*/
      DMAMUX->CHCFG[3]=0;          /*DMAͨ��3Ϊǰ��Ա�����������ݽ���,ʹ��DMAͨ��3*/


      DMAMUX->CHCFG[15]=DMAMUX_CHCFG_ENBL(1) | DMA_CAN0_SOURCE;         /*DMAͨ��15ΪCAN0����FIFO,ʹ��DMAͨ��15*/

      DMAMUX->CHCFG[12]=DMAMUX_CHCFG_ENBL(1) | DMA_FLEXIO_SHIFTER0_SOURCE;         /*DMAͨ��12ΪFLEX_IO����,ʹ��DMAͨ��12*/
	
      DMAMUX->CHCFG[0]=DMAMUX_CHCFG_ENBL(1) | FMT0_DMA_SOURCE;         /*DMAͨ��0Ϊ��ʻԱ�����������ݽ���,ʹ��DMAͨ��0*/
      DMAMUX->CHCFG[1]=DMAMUX_CHCFG_ENBL(1) | FTM1_CH1_DMA_SROUCE;     /*DMAͨ��1Ϊ��ʻԱ�����������ݽ���,ʹ��DMAFTM1ģ��ͨ��1 DMAԴ*/

      DMAMUX->CHCFG[3]=DMAMUX_CHCFG_ENBL(1) | FTM1_CH5_DMA_SROUCE;     /*DMAͨ��1Ϊ��ʻԱ�����������ݽ���,ʹ��DMAFTM1ģ��ͨ��7 DMAԴ*/
      DMAMUX->CHCFG[2]=DMAMUX_CHCFG_ENBL(1) | FTM1_CH7_DMA_SROUCE;     /*DMAͨ��1Ϊ��ʻԱ�����������ݽ���,ʹ��DMAFTM1ģ��ͨ��5 DMAԴ*/

//      DMAMUX->CHCFG[12]=DMAMUX_CHCFG_ENBL(1) | DMA_SPI0_RECE_SOURCE;    /*SPI0ͨ��RX DMAԴ*/
//      DMAMUX->CHCFG[11]=DMAMUX_CHCFG_ENBL(1) | DMA_SPI0_TRAN_SOURCE;    /*SPI0ͨ��TX DMAԴ*/
      
//      DMAMUX->CHCFG[10]=DMAMUX_CHCFG_ENBL(1) | DMA_SPI1_RECE_SOURCE;    /*SPI0ͨ��RX DMAԴ*/
//      DMAMUX->CHCFG[9]= DMAMUX_CHCFG_ENBL(1) | DMA_SPI1_TRAN_SOURCE;    /*SPI0ͨ��TX DMAԴ*/
      
      /*===========================================����EDMA����==========================================*/
      DMA->CR=DMA_CR_CLM(1) | DMA_CR_ERCA(1) | DMA_CR_EMLM(1);  /*Сѭ��ʹ��*/
      
      /*=========================================EDMA���ó�ʼ��==============================================*/
      DMA->DCHPRI[15]=DMA_DCHPRI_CHPRI(15)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*CAN0ͨ��DMA���ȼ�*/

      DMA->DCHPRI[12]=DMA_DCHPRI_CHPRI(12)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*FLEX_IO����ͨ��DMA���ȼ�*/

      DMA->DCHPRI[0]=DMA_DCHPRI_CHPRI(0)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*��ʻԱ��������ͨ��DMA���ȼ�*/
      DMA->DCHPRI[1]=DMA_DCHPRI_CHPRI(1)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*��ʻԱ��������ͨ��DMA���ȼ�*/
      DMA->DCHPRI[2]=DMA_DCHPRI_CHPRI(2)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*��ʻԱ��������ͨ��DMA���ȼ�*/
      DMA->DCHPRI[3]=DMA_DCHPRI_CHPRI(3)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         	/*��ʻԱ��������ͨ��DMA���ȼ�*/
	
//      DMA->DCHPRI[12]=DMA_DCHPRI_CHPRI(12)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*SPI0 RECEͨ��DMA���ȼ�*/
//      DMA->DCHPRI[11]=DMA_DCHPRI_CHPRI(11)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*SPI0 TRANͨ��DMA���ȼ�*/
//      DMA->DCHPRI[10]=DMA_DCHPRI_CHPRI(10)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);         /*SPI1 RECEͨ��DMA���ȼ�*/
//      DMA->DCHPRI[9] =DMA_DCHPRI_CHPRI(9)| DMA_DCHPRI_DPA(1)| DMA_DCHPRI_ECP(1);          /*SPI1 TRANͨ��DMA���ȼ�*/
      
      /*================================����DMAͨ��15��TCB����(CAN0)=====================================*/
      DMA->TCD[15].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(CAN0_RECE_BUFF_SIZE);        /*����ѭ���ڹ�����CAN0_RECE_BUFF_SIZE��Сѭ��*/
      DMA->TCD[15].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(CAN0_RECE_BUFF_SIZE);        /*����ѭ���ڹ�����CAN0_RECE_BUFF_SIZE��Сѭ��*/
      DMA->TCD[15].NBYTES.MLOFFYES=DMA_TCD_NBYTES_MLOFFYES_NBYTES(16)|DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16)|DMA_TCD_NBYTES_MLOFFYES_SMLOE(1);          /*DMA����һ��16���ֽ�*/
      DMA->TCD[15].SADDR=DMA_TCD_SADDR_SADDR(&CAN0->RAMn[0]);           /*CAN0ͨ��DMA����Դ��ַ*/ 
      DMA->TCD[15].SOFF=4;                           /*DMAһ�ζ�д4���ֽ�*/
      DMA->TCD[15].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,32λ���ݸ�ʽ*/
      DMA->TCD[15].SLAST=DMA_TCD_SLAST_SLAST(-16);    /*��ѭ����ɺ�Դ��ַ�ص�*/
      DMA->TCD[15].DADDR=DMA_TCD_DADDR_DADDR(&Can0ReceBuffer[0]);
      DMA->TCD[15].DOFF=4;                           /*DMAһ�ζ�д4���ֽ�*/
      DMA->TCD[15].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-16*CAN0_RECE_BUFF_SIZE);     /*��ѭ����ɺ�Ŀ���ַ�ص�*/
      DMA->TCD[15].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0);
      
//      /*================================����DMAͨ��14��TCB����(CAN1)=====================================*/
//      DMA->TCD[14].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(CAN1_RECE_BUFF_SIZE);         /*����ѭ���ڹ�����CAN1_RECE_BUFF_SIZE��Сѭ��*/
//      DMA->TCD[14].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(CAN1_RECE_BUFF_SIZE);         /*����ѭ���ڹ�����CAN1_RECE_BUFF_SIZE��Сѭ��*/
//      DMA->TCD[14].NBYTES.MLOFFYES=DMA_TCD_NBYTES_MLOFFYES_NBYTES(16)|DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16)|DMA_TCD_NBYTES_MLOFFYES_SMLOE(1);      /*DMA����һ��16���ֽ�*/
//      DMA->TCD[14].SADDR=DMA_TCD_SADDR_SADDR(&CAN1->RAMn[0]);           /*CAN0ͨ��DMA����Դ��ַ*/ 
//      DMA->TCD[14].SOFF=4;                           /*DMAһ�ζ�д4���ֽ�*/
//      DMA->TCD[14].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,32λ���ݸ�ʽ*/
//      DMA->TCD[14].SLAST=DMA_TCD_SLAST_SLAST(-16);    /*��ѭ����ɺ�Դ��ַ�ص�*/
//      DMA->TCD[14].DADDR=DMA_TCD_DADDR_DADDR(&Can1ReceBuffer[0]);
//      DMA->TCD[14].DOFF=4;                           /*DMAһ�ζ�д4���ֽ�*/
//      DMA->TCD[14].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-16*CAN1_RECE_BUFF_SIZE);     /*��ѭ����ɺ�Ŀ���ַ�ص�*/
//      DMA->TCD[14].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0);
//      
//      /*================================����DMAͨ��13��TCB����(CAN2)=====================================*/
//      DMA->TCD[13].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(CAN2_RECE_BUFF_SIZE);        /*����ѭ���ڹ�����CAN2_RECE_BUFF_SIZE��Сѭ��*/     
//      DMA->TCD[13].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(CAN2_RECE_BUFF_SIZE);        /*����ѭ���ڹ�����CAN2_RECE_BUFF_SIZE��Сѭ��*/
//      DMA->TCD[13].NBYTES.MLOFFYES=DMA_TCD_NBYTES_MLOFFYES_NBYTES(16)|DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16)|DMA_TCD_NBYTES_MLOFFYES_SMLOE(1);          /*DMA����һ��16���ֽ�*/
//      DMA->TCD[13].SADDR=DMA_TCD_SADDR_SADDR(&CAN2->RAMn[0]);           /*CAN0ͨ��DMA����Դ��ַ*/ 
 //     DMA->TCD[13].SOFF=4;                           /*DMAһ�ζ�д4���ֽ�*/
 //     DMA->TCD[13].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,32λ���ݸ�ʽ*/
 //     DMA->TCD[13].SLAST=DMA_TCD_SLAST_SLAST(-16);    /*��ѭ����ɺ�Դ��ַ�ص�*/
  //    DMA->TCD[13].DADDR=DMA_TCD_DADDR_DADDR(&Can2ReceBuffer[0]);
 //     DMA->TCD[13].DOFF=4;                           /*DMAһ�ζ�д4���ֽ�*/
 //     DMA->TCD[13].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-16*CAN2_RECE_BUFF_SIZE);     /*��ѭ����ɺ�Ŀ���ַ�ص�*/
 //     DMA->TCD[13].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0);
            


      DMA->SERQ=DMA_SERQ_SERQ(15);        /*ʹ��CAN0ͨ��DMA����*/
//      DMA->SERQ=DMA_SERQ_SERQ(14);        /*ʹ��CAN1ͨ��DMA����*/
//      DMA->SERQ=DMA_SERQ_SERQ(13);        /*ʹ��CAN2ͨ��DMA����*/


};
#pragma arm section code = "RAMCODE"
/*=================================================EEPROM����д��=====================================================*/
unsigned char  EepromQuickWrite(unsigned int Size)
{
      if(((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK)>>SIM_FCFG1_DEPART_SHIFT)== EEPROM_DEPART)/*���EEPROMû�з�������,64KEFLASH*/
      {     /*===========EEPROMû�н��з�������,��EEPROM���з���==============*/
            /*======================����FLEXRAM�����ڷ���EEPROMģʽ================================================*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*�ȴ���һ��FLASH�������*/
            FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*�����ر���*/
            FTFC->FCCOB[3]=EEPROM_SE_FlEXRAM;   /*����FLEXRAM����ģʽFCCOB0*/
            FTFC->FCCOB[2]=EEPROM_QUICK_WRITE;  /*FLEXRAM����д�����,FCCoB1*/
            FTFC->FCCOB[1]=0;
            FTFC->FCCOB[0]=0;
            FTFC->FCCOB[7]=Size>>8;
            FTFC->FCCOB[6]=Size & 0XFF;
            FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*��ʼִ��FLASH��������,д1���*/
            while(((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) || ((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK) ==0)) __NOP();    /*�ȴ�FLASH�������*/
      };
      return FTFC->FCCOB[6];
};
/*======================================================FLEXRAM��������=============================================*/
void  FlexRamParden(void)
{
      if(((SIM->FCFG1 & SIM_FCFG1_DEPART_MASK)>>SIM_FCFG1_DEPART_SHIFT)!= EEPROM_DEPART)/*���EEPROMû�з�������,64KEFLASH*/
      {     /*===========EEPROMû�н��з�������,��EEPROM���з���==============*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*�ȴ���һ��FLASH�������*/
            FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*�����ر���*/
            FTFC->FCCOB[3]=EEPROM_PGMPART;      /*��EEPROM���з���FCCOB0*/
            FTFC->FCCOB[2]=0;                   /*FCCOB1*/
            FTFC->FCCOB[1]=0;                   /*FCCOB2*/
            FTFC->FCCOB[0]=1;                   /*FCCOB3,ϵͳ��λ�����Զ�װ��EEPROM����*/
            FTFC->FCCOB[7]=EEPROM_EEESIZE;      /*EEPROM��С:2KByte,FCCOB4*/
            FTFC->FCCOB[6]=EEPROM_DEPART;       /*EEPROM��������,64KEFLASH,FCCOB5*/
            FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*��ʼִ��FLASH��������,д1���*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  {};    /*�ȴ��������*/
      };
};
/*========================================����BaseAddress��ַ��Χ��Flash����(4K)=================================================*/
void EraseFlashSector(unsigned int  BaseAddress)
{
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*�ȴ���һ��FLASH�������*/
      FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*�����ر���*/
      FTFC->FCCOB[3]=ERASR_FLASH_SECTOR;        /*����FLASH�����е���������*/
      FTFC->FCCOB[2]=(unsigned char)(BaseAddress>>16);
      FTFC->FCCOB[1]=(unsigned char)(BaseAddress>>8);
      FTFC->FCCOB[0]=(unsigned char)BaseAddress;
      FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*��ʼִ��FLASH��������,д1���*/
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*�ȴ��������*/
};
/*===========================================��FLASH������ַ���=============================================================*/
void  WriteFlash(unsigned int  FlashAddress,unsigned char *ReceDataBuff)
{
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*�ȴ���һ��FLASH�������*/
      FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*�����ر���*/
      FTFC->FCCOB[3]=PROGRAM_FLASH;                               /*��FLASH����һ��д�����*/
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
      FTFC->FSTAT=FTFC_FSTAT_CCIF(1);     /*��ʼִ��FLASH��������,д1���*/
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0)  __NOP();    /*�ȴ��������*/
};
#pragma arm section
/*=================================================д��EEPROM����(32λ)===========================================================*/
void  WriteEepromData(unsigned int *Target,unsigned int  *Source,unsigned int size)
{
unsigned int cycles = size / 4;
unsigned int remain = size % 4;
unsigned int offset=0;
      DISABLE_INTERRUPTS();         /*�ر��������*/
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
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*�ȴ��������*/
      ENABLE_INTERRUPTS();          /*ʹ���������*/
};
/*=================================================д��EEPROM����(16λ)===========================================================*/
void  WriteEepromShort(unsigned short *Target,unsigned short  *Source,unsigned int size)
{
unsigned int offset=0;
      DISABLE_INTERRUPTS();         /*�ر��������*/
	for (int i=0;i<size;i++)
	{
            while ((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK) == 0){};
            *(Target + offset)=*(Source + offset);
            offset++;
      };
      __NOP();
      __NOP();
      while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*�ȴ��������*/
      ENABLE_INTERRUPTS();          /*ʹ���������*/
};

/*==============================================����FLEXRAMΪEEPROM����ģʽ=========================================*/
void  FlexramEeprom(void)
{
      if((FTFC->FCNFG & FTFC_FCNFG_EEERDY_MASK)==0)   /*ϵͳ�ڲ�EEPROMû׼����*/
      {
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {};    /*�ȴ���һ��FLASH�������*/
            FTFC->FSTAT=FTFC_FSTAT_ACCERR(1) | FTFC_FSTAT_FPVIOL(1) |FTFC_FSTAT_RDCOLERR(1);    /*�����ر���*/
            FTFC->FCCOB[3]=EEPROM_SE_FlEXRAM;         /*����FLEXRAM����ģʽFCCOB0*/
            FTFC->FCCOB[2]=EEPROM_EMULATED;           /*FLEXRAM�����ڷ���EEPROMģʽ,FCCOB1*/
            FTFC->FSTAT=FTFC_FSTAT_CCIF(1);           /*��ʼִ��FLASH��������,д1���*/
            while((FTFC->FSTAT & FTFC_FSTAT_CCIF_MASK)==0) {} ;    /*�ȴ��������*/
      }
};
/*==============================================��ʼ��QSPIģ��=====================================================
      ����˵��:
            ��ʼ���͹���SPIģ��,�趨SPIͨѶ��ǰ��Ƶ��Ϊ4MHz,���ͨѶƵ���������ַ�Ƶ����.
      ����:
            Channel:����ʹ�õ�SPIģ��ͨ��,һ��������SPIͨ��(SpiChannel0,SpiChannel0,SpiChannel0)
            DmaMode:����SPI�Ƿ�ʹ��DMAͨѶ,1:����DMA;0:������DMA;
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
      LPSpiPoint->CR=0;       /*���ùر���Ӱ��SPIģ��*/
      LPSpiPoint->IER=0;      /*���ùر�SPIģ���ж���Ӧ*/
      if(DmaMode == 1) LPSpiPoint->DER=0X03;   /*������ӦSPIģ���DMA���书��*/
      else LPSpiPoint->DER=0;       /*��ֹ��ӦSPIģ���DMA���书��*/
      LPSpiPoint->CFGR0=0;    /*��������ƥ���ֹ��FIFOѭ����ֹ�����������ֹ��*/
      LPSpiPoint->CFGR1=0X04000001; /*����Ƭѡʹ�ܡ�Ƭѡ�رպ����Ϊ��̬��Sin������롢SoutΪ�������������ƥ���ֹ������Ƭѡ����Ч���Զ�Ƭѡ������ֹ��SPI��������ģʽ*/
//      LPSpiPoint->CCR=0X03030308;   /*SCKPCS=3+1��PCSSCK=3+1��DBT=3+2��SCKDIV=8+2��4MHz��*/
      LPSpiPoint->CCR=0X13130308;   /*SCKPCS=19+1��PCSSCK=19+1��DBT=3+2��SCKDIV=8+2��4MHz��*/
      LPSpiPoint->FCR=0X00000003;   /*RXWATER=0,Rx��FIFO>0����RX��־��TXWATER=3��TX��FIFO<3����TX��־*/
      LPSpiPoint->CR=0X00000009;    /*ʹ����Ӧ��LPSpiģ��*/
};
/*========================================================InitATD��ʼ��A/Dת��=====================================================================*/
void InitATD(void)
{
      PCC->PCCn[PCC_ADC0_INDEX]=0;
//      PCC->PCCn[PCC_ADC1_INDEX]=0;
      PCC->PCCn[PCC_ADC0_INDEX]=PCC_PCCn_PCS(1) | PCC_PCCn_CGC(1);      /*ʹ��ADC0gonzuoʱ��,PCS-=1*/
//      PCC->PCCn[PCC_ADC1_INDEX]=PCC_PCCn_PCS(1) | PCC_PCCn_CGC(1);      /*ʹ��ADC0gonzuoʱ��,PCS-=1*/
      /*==================================ADC0ģ������=============================================*/
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
      /*==================================ADC1ģ������=============================================*/
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
/*=======================================================����ADC�Զ�У������===================================================================*/
void AutoCalibrationATD(void)
{
      /*========================�Զ�У��ADC0===========================*/
      ADC0->CLPS = 0x00u;
      ADC0->CLP3 = 0x00u;
      ADC0->CLP2 = 0x00u;
      ADC0->CLP1 = 0x00u;
      ADC0->CLP0 = 0x00u;
      ADC0->CLPX = 0x00u;
      ADC0->CLP9 = 0x00u;
      ADC0->SC3 =ADC0->SC3 | 0X80;  /*�����Զ�У��*/
      while(ADC0->SC3 & 0X80){};
      /*========================�Զ�У��ADC1===========================*/
//      ADC1->CLPS = 0x00u;
//      ADC1->CLP3 = 0x00u;
//      ADC1->CLP2 = 0x00u;
//      ADC1->CLP1 = 0x00u;
//      ADC1->CLP0 = 0x00u;
//      ADC1->CLPX = 0x00u;
//      ADC1->CLP9 = 0x00u;
//      ADC1->SC3 =ADC1->SC3 | 0X80;  /*�����Զ�У��*/
//      while(ADC1->SC3 & 0X80){};

};
/*=========================================================��������A/Dת��======================================================================*/
void  StartAtdConver(void)
{
//      /*===============================ATD0ͨ���л�====================================================*/
      ADC0->SC1[0]&=~ADC_SC1_ADCH_MASK;         /* Clear prior ADCH bits */
      ADC0->SC1[0] = ADC_SC1_ADCH(Atd0Pos);     /*����A/Dת��*/
//      /*================================ATD1ͨ���л�====================================================*/
//      ADC1->SC1[0]&=~ADC_SC1_ADCH_MASK;         /* Clear prior ADCH bits */
//      ADC1->SC1[0] = ADC_SC1_ADCH(Atd1Pos);     /*����A/Dת��*/
};

/*======================================================���A/Dת�����===============================================================*/
void  GetAtdResult(void)
{
unsigned  int     TempResult;
      /*=======================================��ȡATD0ģ��A/Dת�������ֵ============================================*/
	TempResult=ADC0->R[0];			/*ȡ����һ��A/Dת�����*/
      switch(Atd0Pos)
      {
            case  UB_VOLTAGE_CHANNEL:        			/*ϵͳ����UB��ѹA/Dת��ͨ��*/
			UbVoltageBuff=UbVoltageBuff + TempResult - UbVoltageValue;
			UbVoltageValue=UbVoltageBuff / ATD_FILTER_SCALE;
			UbVoltage=UbVoltageValue * 900 / 1024;	/*UB��ѹֵ������0.01V*/
                  Atd0Pos=AD2428_VOUT2_CHANNEL;			/*AD2424�����ѹVout��ѹA/Dת��ͨ��*/
                  break;
            case  AD2428_VOUT2_CHANNEL:				/*AD2424�����ѹVout��ѹA/Dת��ͨ��*/
			Ad2428Vout2Buff=Ad2428Vout2Buff + TempResult - Ad2428Vout2Value;
			Ad2428Vout2Value=Ad2428Vout2Buff / ATD_FILTER_SCALE;
			Ad2428Vout2= Ad2428Vout2Value * 125 / 1024;	/*AD2428�����ѹVout2��ѹֵ������0.01V*/
                  Atd0Pos=AD2428_POWER_CHANNEL;			/*AD 2428�����ѹA/Dת��ͨ��*/
                  break;
            case  AD2428_POWER_CHANNEL:          		/*AD 2428�����ѹA/Dת��ͨ��*/
			Ad2428PowerBuff=Ad2428PowerBuff + TempResult - AD2428PowerValue;
			AD2428PowerValue=Ad2428PowerBuff / ATD_FILTER_SCALE;
			Ad2428Power=AD2428PowerValue 	* 125 / 512;		/*AD2428�����ѹֵ������0.01V*/
                  Atd0Pos=UB_VOLTAGE_CHANNEL;        		/*ϵͳ����UB��ѹA/Dת��ͨ��*/
                  break;
      };
};
/*====================================��ʼ��PWM������׽����==============================================================*/
void  InitFtm2Capture(void)
{
      PCC->PCCn[PCC_FTM2_INDEX]=0;
      PCC->PCCn[PCC_FTM2_INDEX]=PCC_PCCn_PCS(1) | PCC_PCCn_CGC(1);      /*ʹ��FTM1ģ��ʱ��,PCS-=1*/
      FTM2->MODE=FTM_MODE_WPDIS(1);       /*FTM1�Ĵ�������ģʽȡ��,������������; FTMEN=0*/
      FTM2->SC =0X00000003;         /* TOIE (Timer Overflow Interrupt Ena) = 0 (default) */
                                    /* CPWMS (Center aligned PWM Select) = 0 (default, up count) */
                                    /* CLKS (Clock source) = 0 (default, no clock; FTM disabled) */
                                    /* PS (Prescaler factor) = 3. Prescaler = 8 */
      FTM2->MOD=0;                  /*��ʱ��ģΪ0*/
      FTM2->CNTIN=0;                /*��ʱ����ֵΪ0*/
      FTM2->FILTER=FTM_FILTER_CH0FVAL(3);       /*2uS���˲�ʱ��*/
      FTM2->COMBINE =0X0000000C;    /* FTM mode settings used: DECAP0=1, DECAPEN0=1, MCOMBINEx=0, COMBINEx=0 */    
      FTM2->SC =FTM2->SC | FTM_SC_CLKS(3);      /*��ʱ����ʼ����*/
}
/*===================================��ʼ��׽PWM��������=================================================================
�������:
      Mode:����ģʽ,0:��׽PWM�ߵ�ƽʱ��;1:��׽PWM����ʱ��,����Ϊ1uS
=======================================================================================================================*/
void  StartFtm2Capture(unsigned int Mode)
{
      if(Mode)    /*��׽���ڳ���*/
      {
            FTM2->CONTROLS[0].CnSC=0X00000014;        /*��׽������,����ģʽ*/      
            FTM2->CONTROLS[1].CnSC=0X00000004;        /*��׽������*/
      }else       /*��׽�ߵ�ƽ����*/
      {
            FTM2->CONTROLS[0].CnSC=0X00000014;        /*��׽������,����ģʽ*/      
            FTM2->CONTROLS[1].CnSC=0X00000008;        /*��׽�½���*/
      };
      if(FTM2->CONTROLS[0].CnSC & FTM_CnSC_CHF_MASK) FTM2->CONTROLS[0].CnSC=FTM2->CONTROLS[0].CnSC & (~FTM_CnSC_CHF_MASK);  /*�����־λ*/
      if(FTM2->CONTROLS[1].CnSC & FTM_CnSC_CHF_MASK) FTM2->CONTROLS[1].CnSC=FTM2->CONTROLS[1].CnSC & (~FTM_CnSC_CHF_MASK);  /*�����־λ*/
};
/*======================================��PWM��������================================================================
�������:
      Mode:����ģʽ,0:��׽PWM�ߵ�ƽʱ��;1:��׽PWM����ʱ��,����Ϊ1uS
=======================================================================================================================*/
void  ReadFtm2CaptureData(unsigned int Mode)
{
//unsigned short      TempC0V,TempC1V;
//      if((FTM2->STATUS & 0X03)==3)       /*�ɹ�����˫�ز�׽*/
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
/*===================================================����LPSPI����==============================================================
��������� Channel��LPSPIͨ���ţ�ȡֵ��ΧSpiChannel0 ,SpiChannel1 ,SpiChannel2��
            Buff��Ҫ�������ݵĻ�������
            Size��Ҫ�������ݵ��ֽ�����
            CsUnmber:Ƭѡ���룬0-3��ʾ��PCS0-PCS3��
            Cpol/Cpha��LPSPI�������ݵ���λ��
===============================================================================================================================*/
void  TransLPSpi(SPICHANNEL Channel,unsigned char *Buff,unsigned char Size,unsigned char CsNumber,unsigned char Cpol,unsigned char Cpha)
{
unsigned char     *TempPoint;
unsigned    int   *TempTcr;
unsigned char     TempSize,TempNumber;
      if((Size <= 16) && (Size > 0))   /*SPIһ����ഫ��16���ֽ�,4��������*/
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
                  memcpy(TempPoint,Buff+TempNumber,4);        /*�����ݴ��͵����ͻ�����*/
                  TempNumber=TempNumber+4;
            };
            TempSize=(Size * 8) - 1;            /*����SPI�������ݵ�λ��*/
            /*====����SPI��������,CPOL;CPHA;PRESCALE=3;PCS;BYSW=1:�ֽ�λ�ý���;WIDTH=0====*/
            *TempTcr=(Cpol << 31) + (Cpha << 30) + 0X18400000 +  (CsNumber << 24) + TempSize;
      };
};
/*===================================================��LPSPI��������===========================================================
��������� Channel��LPSPIͨ���ţ�ȡֵ��ΧSpiChannel0 ,SpiChannel1 ,SpiChannel2��
            Buff��Ҫ��ȡ���ݵĻ�������
            Size��Ҫ��ȡ���ݵ��ֽ�����
==============================================================================================================================*/
void  ReadLPSpi(SPICHANNEL Channel,unsigned char *Buff,unsigned char Size)
{
unsigned char     *TempPoint;
unsigned char     TempNumber;

            if((Size <= 16) && (Size > 0))   /*SPIһ����ഫ��16���ֽ�,4��������*/
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
/*==============================================���LPSPI���ջ�����(FIFO)========================================================
��������� Channel��LPSPIͨ���ţ�ȡֵ��ΧSpiChannel0 ,SpiChannel1 ,SpiChannel2��
  ����ֵ:  ���ջ������е����һ������.
================================================================================================================================*/
unsigned int  ClearLPSpiReceFIFO(SPICHANNEL Channel)
{
LPSPI_Type  *LPSpiPoint;
unsigned int      TempValue;  /*��ʱ����*/
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
      while((LPSpiPoint->RSR & 0X02) == 0)      /*���ջ�����(FIFO)����*/
      {
            TempValue=LPSpiPoint->RDR;
      };
      return TempValue;
};
/*===========================================ͨ��DMA��ʽ������EEPROM����SPIͨѶ==========================================
����˵��: 
      �ú������SPIͨѶ,ʹ��DMA��ʽ,һ�δ�����������ͨ��SPI���͸�EEPROM.
����:
      Tran:SPI�������ݻ�������ַ.
      Rece:SPI�������ݻ�������ַ.
      Size:һ�δ��͵���������(Byte����)
=========================================================================================================================*/
void EepromSpiDma(unsigned char *Tran,unsigned char *Rece,int ByteSize)
{
int   TempCount;
int   TempSize;
      if(ByteSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {
            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
            else TempCount=ByteSize /4 ;
            /*================================����DMAͨ��12��TCB����(SPI1 RECE)================================*/
            DMA->TCD[10].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(TempCount);        /*����ѭ���ڹ�����1��Сѭ��*/     
            DMA->TCD[10].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(TempCount);        /*����ѭ���ڹ�����1��Сѭ��*/
            DMA->TCD[10].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(4) ;  
            DMA->TCD[10].SADDR=DMA_TCD_SADDR_SADDR(&LPSPI1->RDR);           /*SPI1ͨ��DMA����Դ��ַ(SPI1���ռĴ���)*/ 
            DMA->TCD[10].SOFF=0;    /*Դ��ַƫ����*/
            DMA->TCD[10].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,32λ���ݸ�ʽ*/
            DMA->TCD[10].SLAST=0;    /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[10].DADDR=DMA_TCD_DADDR_DADDR(Rece);     /*SPI1ͨ��DMA����Ŀ���ַ(SPI1���ջ�����)*/
            DMA->TCD[10].DOFF=4;    /*Ŀ���ַƫ����*/
            DMA->TCD[10].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-4 * TempCount);     /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[10].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(1)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            /*================================����DMAͨ��11��TCB����(SPI1 TRAN)================================*/
            DMA->TCD[9].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(TempCount);        /*����ѭ���ڹ�����1��Сѭ��*/     
            DMA->TCD[9].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(TempCount);        /*����ѭ���ڹ�����1��Сѭ��*/
            DMA->TCD[9].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(4) ;  
            DMA->TCD[9].SADDR=DMA_TCD_SADDR_SADDR(Tran);       /*SPI1ͨ��DMA����Դ��ַ(SPI1����)*/ 
            DMA->TCD[9].SOFF=4;          /*Դ��ַƫ����*/
            DMA->TCD[9].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,32λ���ݸ�ʽ*/
            DMA->TCD[9].SLAST=DMA_TCD_DLASTSGA_DLASTSGA(-4 * TempCount);         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[9].DADDR=DMA_TCD_DADDR_DADDR(&LPSPI1->TDR);       /*SPI1ͨ��DMA����Ŀ���ַ(SPI1���ջ�����)*/
            DMA->TCD[9].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[9].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[9].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(1)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            TempSize=(ByteSize * 8) - 1;              /*����SPI�������ݵ�λ��*/
            LPSPI1->TCR =0X18400000 + TempSize;       /*����SPI0��������,CPOL=0;CPHA=0;PRESCALE=3;PCS=PCS0;BYSW=1:�ֽ�λ�ý���;WIDTH=0*/
            
            DMA->SERQ=DMA_SERQ_SERQ(10);        /*ʹ��SPI1 RECEͨ��DMA����*/
            DMA->SERQ=DMA_SERQ_SERQ(9);         /*ʹ��SPI1 TRANͨ��DMA����*/
      };
};
/*=====================================================��ʼ��FTM0�ĸ�������===============================================
����˵��: 
      �ú�����ɳ�ʼ��FTM0�ĸ�������.
����:
      ��
����ֵ��
			��
=========================================================================================================================*/
void FTM0_init(void)
{
	/**
	 * FTM0 Clocking:
	 * ==================================================
	 */
	PCC->PCCn[PCC_FTM0_INDEX] &= ~PCC_PCCn_CGC_MASK; 	/* Ensure clk disabled for config 	*/
	//ʹ��FTM0ʱ��
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
/*=====================================================��ʼ��FTM0�ĸ�������===============================================
����˵��: 
      �ú�����ɳ�ʼ��FTM1�ĸ�������.
����:
      ��
����ֵ��
			��
=========================================================================================================================*/
void FTM1_init(void)
{
	/**
	 * FTM0 Clocking:
	 * ==================================================
	 */
	PCC->PCCn[PCC_FTM1_INDEX] &= ~PCC_PCCn_CGC_MASK; 	/* Ensure clk disabled for config 	*/
	//ʹ��FTM0ʱ��
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
/*======================================================================FTMģ�鿪ʼ����===================================================
Module��ģ��ţ�0��1��2
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

/*===============================================================��ʼ��FTM0 PWM����======================================================
����˵��: 
      �ú�����ɳ�ʼ��FTM0�ĸ�������,��ҪΪռ�ձ�.
����:
	Channel:ͨ���ţ�0��1��2��3��4��5��6��7
      duty:ռ�ձȣ�0-100��
����ֵ��
			��
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
			FTM0->CONTROLS[5].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK;  	/* FTM0 ch6: edge-aligned PWM, low true pulses ��EN_DMA	*/
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

/*===============================================================��ʼ��FTM0 PWM����======================================================
����˵��: 
      �ú�����ɳ�ʼ��FTM1�ĸ�������,��ҪΪռ�ձ�.
����:
	Channel:ͨ���ţ�0��1��2��3��4��5��6��7
      duty:ռ�ձȣ�0-100��
����ֵ��
			��
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
			FTM1->CONTROLS[1].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK|0X41;  	/* FTM0 ch6: edge-aligned PWM, low true pulses 	��Enable DMA*/
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
			FTM1->CONTROLS[5].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK |0X41;  	/* FTM0 ch6: edge-aligned PWM, low true pulses ��Enable DMA	*/
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
			FTM1->CONTROLS[7].CnSC = FTM_CnSC_MSB_MASK | FTM_CnSC_ELSB_MASK|0X41 ;  	/* FTM0 ch6: edge-aligned PWM, low true pulses ��Enable DMA	*/
															/* CHIE (Chan Interrupt Ena) = 0 (default) 			*/
															/* MSB:MSA (chan Mode Select)=0b10, Edge Align PWM		*/																																			/* ELSB:ELSA (chan Edge/Level Select)=0b10, low true 	*/
			FTM1->CONTROLS[7].CnV =  (FTM1->MOD+1)*duty/100;				/* 0~100 FTM0 ch1 compare value (0~100%:duty cycle) */
		break;
	};
}
/*=============================================================FMT0��DMA0���ݴ���==========================================================================*/
void Fmt0Dma0Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT0��DMA0���ݴ���*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {

            /*================================����DMAͨ��0��TCB����================================*/
            DMA->TCD[0].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/     
            DMA->TCD[0].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/
            DMA->TCD[0].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA����һ�Ρ�2���ֽ�����*/
            DMA->TCD[0].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[0].SOFF=2;          /*Դ��ַƫ����*/
            DMA->TCD[0].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[0].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[0].DADDR=DMA_TCD_DADDR_DADDR(&(FTM0->CONTROLS[7].CnV));       /*SPI0ͨ��DMA����Ŀ���ַ(FTM0-CH7���ջ�����)*/
            DMA->TCD[0].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[0].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[0].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
//            DMA->TCD[0].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(1)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(0);        
		
      };
};
/*========================================================FMT1��DMA1���ݴ���============================================================================*/
void Fmt1Dma1Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT1��DMA1���ݴ���*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {
//            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
//            TempCount=DataSize ;

            /*================================����DMAͨ��0��TCB����================================*/
            DMA->TCD[1].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/     
            DMA->TCD[1].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/
            DMA->TCD[1].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA����һ�Ρ�2���ֽ�����*/
            DMA->TCD[1].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[1].SOFF=2;          /*Դ��ַƫ����*/
            DMA->TCD[1].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[1].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[1].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[1].CnV));       /*SPI0ͨ��DMA����Ŀ���ַ(FTM0-CH5���ջ�����)*/
            DMA->TCD[1].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[1].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[1].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(1);        
		
      };
};
/*========================================================FMT1��DMA2���ݴ���============================================================================*/
void Fmt1Dma2Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT1��DMA1���ݴ���*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {
//            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
//            TempCount=DataSize ;

            /*================================����DMAͨ��0��TCB����================================*/
            DMA->TCD[2].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/     
            DMA->TCD[2].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/
            DMA->TCD[2].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA����һ�Ρ�2���ֽ�����*/
            DMA->TCD[2].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[2].SOFF=2;          /*Դ��ַƫ����*/
            DMA->TCD[2].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[2].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[2].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[7].CnV));       /*SPI0ͨ��DMA����Ŀ���ַ(FTM0-CH5���ջ�����)*/
            DMA->TCD[2].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[2].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[2].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(2);        
		
      };
};
/*========================================================FMT1��DMA3���ݴ���============================================================================*/
void Fmt1Dma3Tran(unsigned char *Tran,unsigned int DataSize)				/*FMT1��DMA1���ݴ���*/
{
//int   TempSize;
      if(DataSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {
//            if( ByteSize % 4) TempCount=(ByteSize /4) + 1;
//            TempCount=DataSize ;

            /*================================����DMAͨ��0��TCB����================================*/
            DMA->TCD[3].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/     
            DMA->TCD[3].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/
            DMA->TCD[3].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA����һ�Ρ�2���ֽ�����*/
            DMA->TCD[3].SADDR=DMA_TCD_SADDR_SADDR(Tran);       
            DMA->TCD[3].SOFF=2;          /*Դ��ַƫ����*/
            DMA->TCD[3].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[3].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[3].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[5].CnV));       /*SPI0ͨ��DMA����Ŀ���ַ(FTM0-CH5���ջ�����)*/
            DMA->TCD[3].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[3].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[3].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);

            
            DMA->SERQ=DMA_SERQ_SERQ(3);        
		
      };
};
/*===================================================================˾��������������==============================================================================
LeftSound�����������ݻ�����
RightSound�����������ݻ�����
DataSize����������������
==================================================================================================================================================================*/
void	DriverMusicOutput(unsigned char *LeftSound,unsigned	char *RightSound,unsigned int DataSize)	
{
	if(DataSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {
		/*=============================================������DMA����=========================================*/
            DMA->TCD[0].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/     
            DMA->TCD[0].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/
            DMA->TCD[0].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA����һ�Ρ�2���ֽ�����*/
            DMA->TCD[0].SADDR=DMA_TCD_SADDR_SADDR(LeftSound);       
            DMA->TCD[0].SOFF=2;          /*Դ��ַƫ����*/
            DMA->TCD[0].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[0].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[0].DADDR=DMA_TCD_DADDR_DADDR(&(FTM0->CONTROLS[7].CnV));       /*SPI0ͨ��DMA����Ŀ���ַ(FTM0-CH7���ջ�����)*/
            DMA->TCD[0].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[0].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[0].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
		/*=============================================������DMA����=========================================*/
            DMA->TCD[1].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/     
            DMA->TCD[1].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����DataSize��Сѭ��*/
            DMA->TCD[1].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(2) ;  		 /*DMA����һ�Ρ�2���ֽ�����*/
            DMA->TCD[1].SADDR=DMA_TCD_SADDR_SADDR(RightSound);       
            DMA->TCD[1].SOFF=2;          /*Դ��ַƫ����*/
            DMA->TCD[1].ATTR=DMA_TCD_ATTR_DSIZE(1) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[1].SLAST=DMA_TCD_DLASTSGA_DLASTSGA((-2 * (int)DataSize));         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[1].DADDR=DMA_TCD_DADDR_DADDR(&(FTM1->CONTROLS[1].CnV));       /*SPI0ͨ��DMA����Ŀ���ַ(FTM0-CH5���ջ�����)*/
            DMA->TCD[1].DOFF=0;          /*Ŀ���ַƫ����*/
            DMA->TCD[1].DLASTSGA=0;      /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[1].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
		/*================================================ʹ��DMA=========================================*/
		DMA->SERQ=DMA_SERQ_SERQ(0);        /*ʹ��SPI0 TRANͨ��DMA����*/
		DMA->SERQ=DMA_SERQ_SERQ(1);        /*ʹ��SPI0 TRANͨ��DMA����*/
	};
};

/*=============================================================��ʼ��FLEX_IO -->IIS========================================================
IIS���������������룩��
	FLEXIO_D3��IIS��������PIN��
	FLEXIO_D4��IIS����ͬ����SYNC����PIN����������ʶ��
	FLEXIO_D5��IIS������λʱ������PIN��
==========================================================================================================================================*/
void	FlexIOInit(void)
{
	/*===================================ʹ��FLEX_IOʱ��=========================================*/	
	PCC->PCCn[PCC_FlexIO_INDEX] = 0;                /* Disable clocks to modify PCS ( default) */
	PCC->PCCn[PCC_FlexIO_INDEX] = 0xC6000000;       /* Enable PCS=SPLL_DIV2 (40 MHz func'l clock) */
/*==============================================================FLEX_IO����1===========================================================*/		
//	FLEXIO->SHIFTCFG[0] = FLEXIO_SHIFTCFG_SSTART(0)				/*Startλ��Ч���������ڵ�һ����λ��������*/
//                               | FLEXIO_SHIFTCFG_SSTOP(0)
//                               | FLEXIO_SHIFTCFG_INSRC(0);			/*PIN*/
//	FLEXIO->SHIFTCTL[0] = FLEXIO_SHIFTCTL_SMOD(1)				/*Receive����ģʽ*/
//                                | FLEXIO_SHIFTCTL_PINPOL(0)			/*�����������ģʽ*/
//					  | FLEXIO_SHIFTCTL_PINSEL(3)			/*FLEX_IO:D3�����ݲɼ��˿�*/
//                                | FLEXIO_SHIFTCTL_PINCFG(0)			/*�����������ģʽ*/
//                                | FLEXIO_SHIFTCTL_TIMPOL(1)			/*�½���������λ*/
//                                | FLEXIO_SHIFTCTL_TIMSEL(0);		/*ѡ��0�Ŷ�ʱ�������λ*/
	/*==================================================Configure SCK timer =========================================================*/
//	FLEXIO->TIMCMP[0] = 63;								/*��FLEXIO->TIMCMP[0]+1��/2���ܵ�����λ����H/L����������*/
//	FLEXIO->TIMCFG[0] = FLEXIO_TIMCFG_TSTART(0)				/*��ʱ��û����ʼλ*/
//                            | FLEXIO_TIMCFG_TSTOP(0)				/*��ʱ��û��ֹͣλ*/
//                            | FLEXIO_TIMCFG_TIMENA(0)				/*��ʱ��ʱ����Ч*/
//                            | FLEXIO_TIMCFG_TIMDIS(0)				/*��ʱ��ʱ����Ч*/
//                            | FLEXIO_TIMCFG_TIMRST(6)				/*��ʱ�����ⲿ���������ظ�λ*/
//                           | FLEXIO_TIMCFG_TIMDEC(2)				/*��ʱ���ݼ���������PIN��˫�أ�����λ�Ĵ�����Ч*/
//                            | FLEXIO_TIMCFG_TIMOUT(0);			/*��ʱ�������*/
//	FLEXIO->TIMCTL[0] = FLEXIO_TIMCTL_TIMOD(3)				/*16λ��ʱ��ģʽ*/
//			          | FLEXIO_TIMCTL_PINSEL(5)				/*��ʱ���ݼ��ź����룬FLEX_IO:D5*/
//                            | FLEXIO_TIMCTL_PINPOL(0)				/*����ģʽ����ע*/
//                            | FLEXIO_TIMCTL_PINCFG(0)				/*��ʱ��û�����*/
//                            | FLEXIO_TIMCTL_TRGSRC(1)				/*��ʱ���ڲ�����*/
//                            | FLEXIO_TIMCTL_TRGPOL(0)				/*��ʱ���޴������*/
//                            | FLEXIO_TIMCTL_TRGSEL(8);			/*��ʱ��������PIN��FLEX_IO:D4*/
/*==============================================================FLEX_IO����2(OK)===========================================================*/	
	FLEXIO->SHIFTCFG[1] = 0X0;
	FLEXIO->SHIFTCTL[1] = 0X01030602;		/*D6���IIS,Timer1,��������λ���*/
	
	FLEXIO->SHIFTCFG[0] = 0X0;
	FLEXIO->SHIFTCTL[0] = 0X00800301;		/*D3����IIS,timer0���½�����λ����*/
//	FLEXIO->SHIFTCTL[0] = 0X00100301;		/*D3����IIS,timer0����������λ����*/

	/* Configure SCK timer */
	FLEXIO->TIMCMP[0] = 63;								/*��FLEXIO->TIMCMP[0]+1��/2���ܵ�����λ����H/L����������*/
	FLEXIO->TIMCFG[0] = 0X00202500;
	FLEXIO->TIMCTL[0] = 0X0B400583;		/*IISʱ���ź�D5,��ʱ��2�������*/
	
	FLEXIO->TIMCMP[1] = 63;								/*��FLEXIO->TIMCMP[0]+1��/2���ܵ�����λ����H/L����������*/
	FLEXIO->TIMCFG[1] = 0X00202500;
	FLEXIO->TIMCTL[1] = 0X03400583;		/*IISʱ���ź�D5*/

	FLEXIO->TIMCMP[2] = 0;								/*��FLEXIO->TIMCMP[0]+1��/2���ܵ�����λ����H/L����������*/
	FLEXIO->TIMCFG[2] = 0X00206400;
	FLEXIO->TIMCTL[2] = 0x0A400483;		/*IISʱ���ź�D5*/
/*==============================================================FLEX_IO����3===========================================================*/		
//	FLEXIO->SHIFTCFG[0] = FLEXIO_SHIFTCFG_SSTART(0)				/*Startλ��Ч���������ڵ�һ����λ��������*/
//					| FLEXIO_SHIFTCFG_SSTOP(0)
//					| FLEXIO_SHIFTCFG_INSRC(0);			/*PIN*/
//	FLEXIO->SHIFTCTL[0] = FLEXIO_SHIFTCTL_SMOD(1)				/*Receive����ģʽ*/
//					| FLEXIO_SHIFTCTL_PINPOL(0)			/*�����������ģʽ*/
//					| FLEXIO_SHIFTCTL_PINSEL(3)			/*FLEX_IO:D3�����ݲɼ��˿�*/
//					| FLEXIO_SHIFTCTL_PINCFG(0)			/*����ģʽ,�����ֹ*/
//					| FLEXIO_SHIFTCTL_TIMPOL(1)			/*�½���������λ*/
//					| FLEXIO_SHIFTCTL_TIMSEL(0);		/*ѡ��0�Ŷ�ʱ�������λ*/
//	/*==================================================Configure SCK timer =========================================================*/
//	FLEXIO->TIMCMP[0] = 33;								/*��FLEXIO->TIMCMP[0]+1��/2���ܵ�����λ����H/L����������*/
//	FLEXIO->TIMCFG[0] = FLEXIO_TIMCFG_TSTART(0)				/*��ʱ��û����ʼλ*/
//					| FLEXIO_TIMCFG_TSTOP(0)			/*��ʱ��û��ֹͣλ*/
//					| FLEXIO_TIMCFG_TIMENA(6)			/*��ʱ��ʹ�ܲ���λ�����������ź�������D4��SYNC*/
//					| FLEXIO_TIMCFG_TIMENA(5)			/*��ʱ��ʹ�ܲ���λ�����������źŸߵ�ƽD4��SYNC����λ�ź�������*/
//					| FLEXIO_TIMCFG_TIMDIS(2)			/*��ʱ����ֹ��������ʱ���Ƚ���ɺ�*/
//					| FLEXIO_TIMCFG_TIMRST(0)			/*��ʱ����λ��������Զ����λ*/
//					| FLEXIO_TIMCFG_TIMDEC(2)			/*��ʱ���ݼ���������PIN��D3��˫�أ�����λ�Ĵ�����Ч*/
//					| FLEXIO_TIMCFG_TIMOUT(0);			/*��ʱ����ʼ�����1�����ܶ�ʱ����λӰ��*/
//	FLEXIO->TIMCTL[0] = FLEXIO_TIMCTL_TIMOD(3)				/*16λ��ʱ��ģʽ*/
//					| FLEXIO_TIMCTL_PINPOL(0)			/*����ģʽ����ע*/
//					| FLEXIO_TIMCTL_PINSEL(5)			/*��ʱ���ݼ��ź����룬FLEX_IO:D5*/
//					| FLEXIO_TIMCTL_PINCFG(0)			/*��ʱ��Ϊ����ģʽ�������ֹ*/
//					| FLEXIO_TIMCTL_TRGSRC(1)			/*��ʱ���ڲ�����*/
//					| FLEXIO_TIMCTL_TRGPOL(0)			/*��ʱ���޴������*/
//					| FLEXIO_TIMCTL_TRGSEL(8);			/*��ʱ��������PIN��FLEX_IO:D4*/
/*=================================================================ʹ��DMA=============================================================*/
	FLEXIO->SHIFTSIEN=0X01;				/*��λ�Ĵ���0 �ж�ʹ��*/
	FLEXIO->SHIFTSDEN=0X01;				/*��λ�Ĵ���0 DMAʹ��*/
	FLEXIO->CTRL=1;					/*ʹ��FLEX_IOģ��ʹ��*/
};

/*====================================================================================================================================*/
void FlexioDmaRece(unsigned char *ReceBuff,int DataSize)				/*Flex_IO��DMA12���ݴ���*/
{
      if(DataSize>0)    /*SPI���͵����ݱ������һ���ֽ�*/
      {

            /*================================����DMAͨ��0��TCB����================================*/
            DMA->TCD[12].BITER.ELINKNO=DMA_TCD_BITER_ELINKNO_BITER(DataSize);        /*����ѭ���ڹ�����1��Сѭ��*/     
            DMA->TCD[12].CITER.ELINKNO=DMA_TCD_CITER_ELINKNO_CITER(DataSize);        /*����ѭ���ڹ�����1��Сѭ��*/
            DMA->TCD[12].NBYTES.MLOFFNO=DMA_TCD_NBYTES_MLOFFNO_NBYTES(4) ;  		 /*DMA����һ�Ρ�4���ֽ�����*/
            DMA->TCD[12].SADDR=DMA_TCD_SADDR_SADDR(&FLEXIO->SHIFTBUFBIS[0]);       
            DMA->TCD[12].SOFF=0;          /*Դ��ַƫ����*/
            DMA->TCD[12].ATTR=DMA_TCD_ATTR_DSIZE(2) | DMA_TCD_ATTR_DMOD(0) |DMA_TCD_ATTR_SSIZE(2) | DMA_TCD_ATTR_SMOD(0);     /*Դ/Ŀ�ĵ�ַģ����,16λ���ݸ�ʽ*/
            DMA->TCD[12].SLAST=0;         /*��ѭ����ɺ�Դ��ַ�ص�*/
            DMA->TCD[12].DADDR=DMA_TCD_DADDR_DADDR(ReceBuff);       /**/
            DMA->TCD[12].DOFF=4;          /*Ŀ���ַƫ����*/
            DMA->TCD[12].DLASTSGA=DMA_TCD_DLASTSGA_DLASTSGA(-4*DataSize);     /*��ѭ����ɺ�Ŀ���ַ�ص�*/
            DMA->TCD[12].CSR=DMA_TCD_CSR_START(0) |DMA_TCD_CSR_INTMAJOR(0)|DMA_TCD_CSR_INTHALF(0) | DMA_TCD_CSR_DREQ(0)|DMA_TCD_CSR_ESG(0) |DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_MAJORELINK(0)|DMA_TCD_CSR_BWC(0) | DMA_TCD_CSR_DONE(0);
		DMA->SERQ=DMA_SERQ_SERQ(12);        /*ʹ��FLEX_IOͨ��DMA����*/
		
      };
};
/*==========================================================================================================================*/
unsigned	int	GetIisDmaPosition(unsigned int TotalBufferSize)		/*����DMA����IIS�����ڻ������е�λ��*/
{
	return TotalBufferSize-DMA->TCD[12].CITER.ELINKNO;
};
/*==========================================================================================================================*/
void	DisableDma(unsigned int Channel)				/*�ر���Ӧ��DMAͨ��*/
{
	if(Channel <= 15)
		DMA->CERQ=DMA_CERQ_CERQ(Channel);
};
/*==========================================================================================================================*/
unsigned	int	GetSoundDrvPosition(unsigned int TotalBufferSize)		/*����DMA�������������ڻ������е�λ��*/	
{
	return TotalBufferSize-DMA->TCD[0].CITER.ELINKNO;
};
	



