/*******************************************************************************

  File Name   : ccp_can_interface.h
  Date        : 27.07.2001
  Version     : 1.0
  Desciption  : additional functions for CCP


*******************************************************************************/

#include "CCPPAR.H"                             // CPP config file
// -----------------------------------------------------------------------------
// SENDING an CRM-DTO when receiving an CRO
// -----------------------------------------------------------------------------
BYTE ccpSend( BYTEPTR msg );
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// CONVERT pointer
// -----------------------------------------------------------------------------
MTABYTEPTR ccpGetPointer( BYTE addr_ext, DWORD addr );        // get Pointer into

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
CCP_DAQBYTEPTR ccpGetDaqPointer(CCP_BYTE addr_ext, CCP_DWORD addr);
#endif



/*----------------------------------------------------------------------------*/
/* Check addresses for valid write access */
/* Used only if Write Protection is required */
/* Returns false if access denied */
#if defined ( CCP_WRITE_PROTECTION )
CCP_BYTE ccpCheckWriteAccess(CCP_MTABYTEPTR a, CCP_BYTE s);
#endif





/*----------------------------------------------------------------------------*/
/* Example: Flash Programming */
/* Used only if integrated Flash Programming is required */

#if defined ( CCP_PROGRAM )

# if defined ( CCP_BOOTLOADER )
# else

#include "flash.h"

void ccpFlashClear(CCP_MTABYTEPTR a, CCP_DWORD size);

CCP_BYTE ccpFlashProgramm(CCP_BYTEPTR data, CCP_MTABYTEPTR a, CCP_BYTE size);
# endif	/* CCP_BOOTLOADER */
#endif /* CCP_PROGRAM */


/*----------------------------------------------------------------------------*/
/* Example: Calibration CCP_RAM/CCP_ROM Selection */
/* Used if Flash Programming is required */

#if defined ( CCP_CALPAGE ) || defined ( CCP_PROGRAM )
CCP_DWORD ccpGetCalPage(void);

void ccpSetCalPage(CCP_DWORD a);
void ccpInitCalPage(void);

#endif

/*----------------------------------------------------------------------------*/
/* Example: Seed&Key*/
/* Used only if Seed&Key is required */

#if defined ( CCP_SEED_KEY )
CCP_DWORD ccpGetSeed(CCP_BYTE resourceMask);
CCP_BYTE ccpUnlock(CCP_BYTE *key);
#endif


/*----------------------------------------------------------------------------*/
/* Example: EEPROM write access */
/* Used only if required */

#if defined ( CCP_WRITE_EEPROM )

/* Pending EEPROM write cycle */

/* EEPROM write */
/* Return values for ccpCheckWriteEEPROM:
   CCP_WRITE_OK      - EEPROM written
   CCP_WRITE_DENIED  - This address is not in EEPROM
   CCP_WRITE_PENDING - EEPROM write in progress, call ccpSendCrm() when done
   CCP_WRITE_ERROR   - EEPROM write failed
*/
CCP_BYTE ccpCheckWriteEEPROM(CCP_MTABYTEPTR addr, CCP_BYTE size, CCP_BYTEPTR data);

/* Check for EEPROM write finished */
void ccpCheckPendingEEPROM( void );

#endif


#if defined ( CCP_READ_EEPROM )

/* EEPROM read */
/* Return values for ccpCheckReadEEPROM:
   0 (FALSE)    - This address is not in EEPROM
   1 (TRUE)     - EEPROM read
*/
CCP_BYTE ccpCheckReadEEPROM( CCP_MTABYTEPTR addr, CCP_BYTE size, CCP_BYTEPTR data );
#endif



// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// CALCULATE new measuerments
// -----------------------------------------------------------------------------
void ccpUserBackground( void );
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Define
// -----------------------------------------------------------------------------
BYTE ccpDisableNormalOperation( MTABYTEPTR a, WORD s );
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
// Disable/Enable Interrupt Functions
// -----------------------------------------------------------------------------
//void disable_interrupt(); 
//void enable_interrupt();
// -----------------------------------------------------------------------------














// -----------------------------------------------------------------------------
