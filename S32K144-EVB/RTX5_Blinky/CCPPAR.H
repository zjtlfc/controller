/*----------------------------------------------------------------------------
| File:
|   ccppar.h
|
| Project:
|   CCP driver example
|   CANape CAN Calibration Tool
|
| Description:
|   Customization header for ccp.c
|   See CCP.DOC for a complete description of all options
|
 ----------------------------------------------------------------------------*/

#ifndef __CCPPAR_H__
#define __CCPPAR_H__


/*----------------------------------------------------------------------------*/
/* Platform independant types */

#define CCP_INTEL

#define     BYTE        unsigned char
#define     CCP_BYTE    unsigned char
#define     WORD        unsigned short
#define     CCP_WORD    unsigned short
#define     DWORD       unsigned int
#define     CCP_DWORD   unsigned int
#define     BYTEPTR     unsigned char*
#define     MTABYTEPTR  BYTEPTR
#define     ROM
#define     RAM
/*
#define DAQBYTEPTR CCP_BYTEPTR
#define MTABYTEPTR CCP_BYTEPTR
*/


/*----------------------------------------------------------------------------*/
/* Disable/Enable Interrupts */

/* Has to be defined if ccpSendCallBack may interrupt ccpDaq */
#define CCP_DISABLE_INTERRUPT
#define CCP_ENABLE_INTERRUPT

/*----------------------------------------------------------------------------*/
/* CCP parameters */

/* CCP Identifiers and Address */
#define CCP_STATION_ADDR  0X0047          	/* Define CCP_STATION_ADDR in Intel Format */
                                      		/* High/Low byte swapped on motorola machines !!! (0x4700) */

#define CCP_STATION_ID    "BMS_S32K"  		/* Plug&Play station identification */

#define CCP_DTO_ID        0X1f880001           		/* CAN identifier ECU -> Master */
#define CCP_CRO_ID        0X1f880000           		/* CAN identifier Master -> ECU */


/*----------------------------------------------------------------------------*/
/* CCP Data Acuisition Parameters */

#define CCP_DAQ                   	/* Enable synchronous data aquisition in ccpDaq() */
#define CCP_MAX_ODT 25          	/* Number of ODTs in each DAQ lists */
#define CCP_MAX_DAQ 4             	/* Number of DAQ lists */


/*----------------------------------------------------------------------------*/
/* CCP Options */

/* Use the transmit queue in CCP.C */
/* Complete sampling is done in ccpDaq(x) and the messages are written into the queue */
//#define CCP_SEND_QUEUE

/* Indicate queue overruns in the msb of pid */
/* Will be displayed in CANape's status bar if CANAPE.INI: [asap1a] check_overflow=1 */
//#define CCP_SEND_QUEUE_OVERRUN_INDICATION

/* Transmit only one message in one call to ccpDaq() */
#define CCP_SEND_SINGLE

/* Allow an ODT entry size >1 */
/* Not recommended for CANape, this will only need additional RAM space */
#define CCP_ODT_ENTRY_SIZE

/* Use GET_SEED and UNLOCK */
/* This is usually user dependant, just a skeleton here */
/* #define CCP_SEED_KEY */

/* Implement the flash programming feature in the ECU*/
/* This is usually user dependant, just a skeleton here */
/* #define CCP_PROGRAM*/

/* Activate the flash programming kernel download feature */
/* This is a CCP extension for CANape */
/* #define CCP_BOOTLOADER_DOWNLOAD */

/* Implement the memory checksum feature */
/* The checksum will be calculated in ccpBackground() */
/* This may be implementation specific */
#define CCP_CHECKSUM
#define CCP_CHECKSUM_TYPE CCP_WORD

/* Use a 16 bit CRC algorithm */
/* Note:
   This will need additional 512 byte of ROM
   CCP_CHECKSUM_TYPE has to be WORD !
*/
#define CCP_CHECKSUM_CCITT

/* Check for pending CCP commands in ccpBackground() */
/* #ifdef CCP_CMD_NOT_IN_INTERRUPT */

/* Enable Memory Write Protection */
/* #define CCP_WRITE_PROTECTION*/

/* Enable EEPROM Read/Write Access */
/* #define CCP_WRITE_EEPROM */
/* #define CCP_READ_EEPROM */










/*----------------------------------------------------------------------------*/
/* Special values for the CCPsim WIN32 Application */
#if defined(__WIN32__) || defined(WIN32)

  #define CCP_TESTMODE /* Turn on screen output via printf */
  #define CCPPRINT printf

  #define CCP_INTEL

  extern unsigned long gDtoId;
  extern unsigned long gCroId;
  extern unsigned short gStationAddr;
  #undef CCP_CRO_ID
  #define CCP_CRO_ID gCroId
  #undef CCP_DTO_ID
  #define CCP_DTO_ID gDtoId
  #undef CCP_STATION_ADDR
  #define CCP_STATION_ADDR gStationAddr
  #undef CCP_STATION_ID
  #define CCP_STATION_ID "CCPSIM"

  #undef CCP_MAX_ODT
  #undef CCP_MAX_DAQ
  #define CCP_MAX_ODT 10
  #define CCP_MAX_DAQ 5
  #define CCP_ODT_ENTRY_SIZE
  #undef  CCP_SEND_SINGLE
  #undef  CCP_SEND_QUEUE
  #define CCP_SEND_QUEUE_OVERRUN_INDICATION
  #define CCP_SEND_QUEUE_SIZE (CCP_MAX_ODT*CCP_MAX_DAQ)
  #define CCP_SEED_KEY
  #undef  CCP_BOOTLOADER_DOWNLOAD
  #define CCP_PROGRAM
  #define CCP_WRITE_PROTECTION
  #define CCP_CHECKSUM
  #define CCP_CHECKSUM_TYPE CCP_WORD
  #define CCP_CHECKSUM_CCITT

#endif

#endif
