/*******************************************************************************

  File Name   : ccp_can_interface.c
  Date        : 28.09.2001
  Version     : 1.0.1
  Desciption  : additional functions for CCP


*******************************************************************************/


// -----------------------------------------------------------------------------
// INCLUDE FILES
// -----------------------------------------------------------------------------
#include "ccp_can_interface.H"   // additional functions for CCP usage
#include "CCPPAR.H"
#include "S32K144_API.h"
#include "s32_core_cm4.h"
#include "string.h"
// -----------------------------------------------------------------------------
#if defined ( CCP_CALPAGE )
  CCP_BYTE ccpCalPage = 0;
#endif


//#pragma CODE_SEG NON_BANKED
// -----------------------------------------------------------------------------
// SWI Interrupt Service Routine
// -----------------------------------------------------------------------------
//interrupt VectorNumber_Vswi void SWI_ISR(void) 
//{                                       // SWI function,

//}                                       // interrupt vect.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Empty Interrupt Service Routine
// -----------------------------------------------------------------------------
//interrupt void irq_dummy(void) 
//{                                       // dummy function,
//
//}                                       // interrupt vect.

//#pragma CODE_SEG DEFAULT

// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// SENDING an CRM-DTO when receiving an CRO
// -----------------------------------------------------------------------------
BYTE ccpSend( BYTEPTR msg )
{
struct  CAN_MESSAGE_BUFFER  TempCanTran;      
      TempCanTran.IDE=1;      /*扩展帧*/
      TempCanTran.DLC=8;      /*8个字节长度*/
      TempCanTran.SRR=1;      /*默认值:1*/
      TempCanTran.RTR=0;      /*非远程帧*/
      TempCanTran.CAN_ID.EXTEND.ID=CCP_DTO_ID;
      (void)memcpy(TempCanTran.Data,msg,8);
      (void)CANTranQueue(CanChannel0,&TempCanTran);         /*队列发送CAN数据*/
      return 1;
}
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// CONVERT pointer
// -----------------------------------------------------------------------------
MTABYTEPTR ccpGetPointer( BYTE addr_ext, DWORD addr )        // get Pointer into
{                                                            // normal C


#if 0
  #if defined ( V_ENABLE_USE_DUMMY_STATEMENT )
  /* avoid compiler warning due to unused parameters */
  addr_ext = addr_ext;
  #endif

  /* Example C16x: DDP1 used for CCP_RAM/CCP_ROM selection */
  #if defined ( CANBOX ) || defined ( PHYTEC_MM167 )
    #if defined ( CCP_CALPAGE )
    if((ccpCalPage == 1) && (addr >= 0x14000) && (addr < 0x18000))
    { /* CALRAM */
      return (CCP_MTABYTEPTR)(addr + 0x30000UL);
    }
    #endif
  #endif

  return (CCP_MTABYTEPTR)addr;



#endif
                                                         
  return (MTABYTEPTR) addr;
}

// -----------------------------------------------------------------------------
// ccpGetDaqPointer

// Convert a memory address from CCP 8/32bit into a address
// stored in the ODT entries for DAQ.

// This is for reducing memory space used for DAQ lists.
// For example on a 32 bit microcontroller, a DAQ pointer may be stored as 16 bit
// value. DAQ will add the base address CCP_DAQ_BASE_ADDR before dereferencing the
// pointer. This will limit data acquisition to a single 64K memory range, but it
// will save 50% memory used for DAQ lists.

// Note: It must be possible to calculate the final address for DAQ like
//  value = * (CCP_DAQ_BASE_ADDR + addr);
// -----------------------------------------------------------------------------

#if defined ( CCP_DAQ_BASE_ADDR )
CCP_DAQBYTEPTR ccpGetDaqPointer(CCP_BYTE addr_ext, CCP_DWORD addr)
{
  return (CCP_DAQBYTEPTR)(ccpGetPointer(addr_ext, addr) - CCP_DAQ_BASE_ADDR);
}
#endif



/*----------------------------------------------------------------------------*/
/* Check addresses for valid write access */
/* Used only if Write Protection is required */
/* Returns false if access denied */
#if defined ( CCP_WRITE_PROTECTION )
CCP_BYTE ccpCheckWriteAccess(CCP_MTABYTEPTR a, CCP_BYTE s)
{
  /* Protect CCP */
  if(a+s>=(CCP_MTABYTEPTR)&ccp && a<(CCP_MTABYTEPTR)&ccp+sizeof(ccp))
  {
    return 0;
  }

  return 1;
}
#endif





/*----------------------------------------------------------------------------*/
/* Example: Flash Programming */
/* Used only if integrated Flash Programming is required */

#if defined ( CCP_PROGRAM )

# if defined ( CCP_BOOTLOADER )
# else

#include "flash.h"

void ccpFlashClear(CCP_MTABYTEPTR a, CCP_DWORD size)
{
  #if defined ( CANBOX ) || defined ( PHYTEC_MM167 )
    #if defined ( CCP_CALPAGE )
    if(a >= (CCP_MTABYTEPTR)0x40000)
    {
      a -= 0x30000; /* Compensate CCP_RAM/CCP_ROM mapping */
    }
    #endif
  #endif

  CCP_DISABLE_INTERRUPT;
  flashEraseBlock(a);
  CCP_ENABLE_INTERRUPT;
}

CCP_BYTE ccpFlashProgramm(CCP_BYTEPTR data, CCP_MTABYTEPTR a, CCP_BYTE size)
{
  #if defined ( CANBOX ) || defined ( PHYTEC_MM167 )
    #if defined ( CCP_CALPAGE )
    if(a >= (CCP_MTABYTEPTR)0x40000)
    {
      a -= 0x30000; /* Compensate CCP_RAM/CCP_ROM mapping */
    }
    #endif
  #endif

  if(size == 0)
  { /* End of programing sequence */
    /* Software Reset */
    asm swi;     //add by hzh
  }

  while(size > 0)
  {
    CCP_DISABLE_INTERRUPT;
    flashByteWrite(a, *data);
    CCP_ENABLE_INTERRUPT;
    data++;
    a++;
    size--;
  }

  return CCP_WRITE_OK;
}

# endif	/* CCP_BOOTLOADER */
#endif /* CCP_PROGRAM */


/*----------------------------------------------------------------------------*/
/* Example: Calibration CCP_RAM/CCP_ROM Selection */
/* Used if Flash Programming is required */

#if defined ( CCP_CALPAGE ) || defined ( CCP_PROGRAM )

CCP_DWORD ccpGetCalPage(void)
{
  return (CCP_DWORD)ccpCalPage;
}

void ccpSetCalPage(CCP_DWORD a)
{
  ccpCalPage = (CCP_BYTE)a;


}

void ccpInitCalPage(void)
{

}

#endif

/*----------------------------------------------------------------------------*/
/* Example: Seed&Key*/
/* Used only if Seed&Key is required */

#if defined ( CCP_SEED_KEY )

CCP_BYTE ccpResourceMask = 0;
CCP_DWORD ccpLastSeed = 0;

CCP_DWORD ccpGetSeed(CCP_BYTE resourceMask)
{
  ccpResourceMask = resourceMask;

  /* Generate a seed */

  /* Example: */
  /* Optimum would be a number which never appears twice */
  #if defined ( CCP_TIMESTAMPING )
  ccpLastSeed = ccpGetTimestamp() * ccpGetTimestamp();
  return ccpLastSeed;
  #endif

  return 0;
}

CCP_BYTE ccpUnlock(CCP_BYTE *key)
{
  /* Check the key */

  /* Example: */
  ccpLastSeed = (ccpLastSeed>>5) | (ccpLastSeed<<23);
  ccpLastSeed *= 7;
  ccpLastSeed ^= 0x26031961;

  if(*(CCP_DWORD*)key != ccpLastSeed)
  {
    return 0;
  }

  /* Reset resource protection bit */
  return ccpResourceMask;
}
#endif


/*----------------------------------------------------------------------------*/
/* Example: EEPROM write access */
/* Used only if required */

#if defined ( CCP_WRITE_EEPROM )
  #include "eeprom.h"

/* Pending EEPROM write cycle */
CCP_BYTE ccpEEPROMState = 0;

/* EEPROM write */
/* Return values for ccpCheckWriteEEPROM:
   CCP_WRITE_OK      - EEPROM written
   CCP_WRITE_DENIED  - This address is not in EEPROM
   CCP_WRITE_PENDING - EEPROM write in progress, call ccpSendCrm() when done
   CCP_WRITE_ERROR   - EEPROM write failed
*/
CCP_BYTE ccpCheckWriteEEPROM(CCP_MTABYTEPTR addr, CCP_BYTE size, CCP_BYTEPTR data)
{
  /* Check address for EEPROM */
  if(addr<EEPROM_START || addr>EEPROM_END)
  {
    /* Not EEPROM */
    /* Let the CCP driver perform a standard CCP_RAM write access */
    return CCP_WRITE_DENIED;
  }

  /* Alternative 1: */
  /* Initiate EEPROM write */
  /* When finished, call ccpSendCrm() */
  #if defined ( C_CLIENT_BMWAG )

    #if ( EEBUFFER < 5 )
      #error "! CCP will need at least 5 Entries in EEBUFFER !"
    #endif

    if(EECheckBuffer() > (EEBUFFER_MAX - size))
    {
      return CCP_WRITE_ERROR;
    }

    while(size--)
    {
      if(E_OK != EEAddByte((unsigned int) addr++,*data++))
      {
        return CCP_WRITE_ERROR;
      }
    }

    ccpEEPROMState = CCP_WRITE_PENDING;

    return CCP_WRITE_PENDING;

  /* Alternative 2: */
  /* Write to EEPROM here and wait until finished */
  #else

    eeWrite(addr,data,size);
    return CCP_WRITE_OK;

  #endif


}

/* Check for EEPROM write finished */
void ccpCheckPendingEEPROM( void )
{
  #if defined ( C_CLIENT_BMWAG )
  if(ccpEEPROMState == CCP_WRITE_PENDING)
  {
    if(EECheckBuffer() == 0)
    {
      ccpSendCrm();
      ccpEEPROMState = 0;
    }
  }
  #endif
}

#endif


#if defined ( CCP_READ_EEPROM )

/* EEPROM read */
/* Return values for ccpCheckReadEEPROM:
   0 (FALSE)    - This address is not in EEPROM
   1 (TRUE)     - EEPROM read
*/
CCP_BYTE ccpCheckReadEEPROM( CCP_MTABYTEPTR addr, CCP_BYTE size, CCP_BYTEPTR data )
{

  /* Read EEPROM */
  #if defined ( C_CLIENT_BMWAG )
  /* Check address for EEPROM */
  if((addr < EEPROM_START) || (addr > EEPROM_END))
  {
    /* Not EEPROM */
    /* Let the CCP driver perform a standard CCP_RAM read access */
    return 0;
  }

  while(size-- > 0)
  {
    *data++ = EEReadByte(addr++);
  }

  return 1;
  #else
  return 0;
  #endif
}

#endif



// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// CALCULATE new measuerments
// -----------------------------------------------------------------------------
void ccpUserBackground( void )
{

#if 0
  /* Try to retransmit if CAN driver transmit queue is not enabled */
  #if defined  ( C_ENABLE_TRANSMIT_QUEUE )
  #else
  if(ccp.SendStatus & CCP_TX_PENDING)
  {
    if(CanTransmit(CCP_TX_HANDLE) == kCanTxOk)
    {
      ccp.SendStatus &= ~CCP_TX_PENDING;
    }
  }
  #endif

  /* Check if a pending EEPROM write access is finished */
  #if defined ( CCP_WRITE_EEPROM )
  ccpCheckPendingEEPROM();
  #endif

  /* ... */
  /* Insert any other user actions here */
  /* Call ccpSendCrm() to finish pending EEPROM or FLASH cycles */

#endif

}
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Define
// -----------------------------------------------------------------------------
BYTE disable_interrupt_count = 0;       // init counter
BYTE ccpDisableNormalOperation( MTABYTEPTR a, WORD s )
{
      disable_interrupt_count++;            // incr. counter
      DISABLE_INTERRUPTS();         /*关闭所有中断*/
      return 1;
}
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Disable/Enable Interrupt Functions
// -----------------------------------------------------------------------------

//void disable_interrupt() 
//{
//
//  DISABLE_INTERRUPTS();         /*关闭任务调度*/                        // set I-Bit
//}

//void enable_interrupt() 
//{
//
//  if (--disable_interrupt_count==0) 
//  {                                     // if interrupts only one time disabled
//    ENABLE_INTERRUPTS();                       // enable them
//  }
//}
// -----------------------------------------------------------------------------
