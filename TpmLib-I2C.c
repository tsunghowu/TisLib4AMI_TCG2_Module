/*++

Copyright (c) 2005 Intel Corporation. All rights reserved
This software and associated documentation (if any) is furnished
under a license and may only be used or copied in accordance
with the terms of the license. Except as permitted by such
license, no part of this software or documentation may be
reproduced, stored in a retrieval system, or transmitted in any
form or by any means without the express written consent of
Intel Corporation.


Module Name:

  TpmLib.h

Abstract:

  Common code of TPM driver in both PEI and DXE
--*/
//*************************************************************************
// $Header: /Alaska/SOURCE/Modules/TcgNext/Libraries/TisLib/TpmLib.c 1     10/08/13 11:58a Fredericko $
//
// $Revision: 1 $
//
// $Date: 10/08/13 11:58a $
//*************************************************************************
// Revision History
// ----------------
// $Log: /Alaska/SOURCE/Modules/TcgNext/Libraries/TisLib/TpmLib.c $
// 
// 1     10/08/13 11:58a Fredericko
// Initial Check-In for Tpm-Next module
// 
// 2     10/03/13 12:33p Fredericko
// 
// 1     7/10/13 5:50p Fredericko
// [TAG]  		EIP120969
// [Category]  	New Feature
// [Description]  	TCG (TPM20)
// [Files]  		TisLib.cif
// TisLib.mak
// TcgTpm12.h
// TpmLib.h
// TcgCommon.h
// ZTEICTcmOrdinals.h
// TpmLib.c
// TcgCommon.c
// TisLib.sdl
// sha1.h
// INTTcgAcpi.h
// TcgPc.h
// TcmPc.h
// TcgEfiTpm.h
// TcgEFI12.h
// 
// 23    9/08/11 4:46p Fredericko
// [TAG]  		EIPEIP0000
// [Category]  	Improvement
// [Description]  	Added ability to modify Delay Amount for TPM driver
// [Files]  		Tcg.sdl
// Tpmlib.c
// 
// 22    8/26/11 12:31p Fredericko
// [TAG]  		EIP64300 
// [Category]  	Improvement
// [Description]  	Allow variations for TPM driver wait times. 
// [Files]  		Tpmlib.c
// Tcg.sd
// 
// 21    7/07/10 12:09p Fredericko
// Make Wait variable a #define.
// 
// 20    5/19/10 4:50p Fredericko
// Included File Header
// 
// 19    5/18/10 5:39p Fredericko
// Code Beautification and Enhancement
// 
// 18    5/18/10 11:32a Fredericko
// Header modification for AMI code standard.
// 
// 17    5/04/10 3:36p Fredericko
// Source Enhancement. EIP 22208
// 
//*************************************************************************
//*************************************************************************
//<AMI_FHDR_START>
//
// Name:  TpmLib.c
//
// Description: 
//  Contains low level TCG functions
//
//<AMI_FHDR_END>
//*************************************************************************
#include <Efi.h>
#include "AmiTcg\TpmLib.h"
#include <Library/BaseLib.h>
#include<Library/IoLib.h>
#include <token.h>
#include <Library\DebugLib.h>
#include "I2CLib/I2CHc.h"

extern EFI_STATUS CountTime ( IN UINTN	DelayTime,  IN	UINT16	BaseAddr); // only needs to be 16 bit for I/O address)
#define Wait  TPM_DRIVER_WAIT 
#define ACCESS_WAITCOUNT    (750 * 1000 / 100)    // 750MS (750 * 10000 /1000)
#define ACCESS_WAITCOUNTB   (  3 * 1000000/100) // 3 seconds delay

#define NUM_BITS_IN_ACPI_TIMER  24  // Porting
#define MAX_ACPI_TIMER_BITS     32  // Porting

#define TIS_TIMEOUT_A   ACCESS_WAITCOUNT
#define TIS_TIMEOUT_B   ACCESS_WAITCOUNTB
#define TIS_TIMEOUT_C   ACCESS_WAITCOUNT
#define TIS_TIMEOUT_D   ACCESS_WAITCOUNT

extern 
UINT8
EFIAPI
TpmReadByte (
  IN  UINTN  Address
  );

extern 
VOID
TpmWriteByte (
  IN UINTN  Address,
  IN UINT8  Data
  );

EFI_STATUS
EFIAPI
TisPcWaitRegisterBits (
  IN      UINT8                     *Register,
  IN      UINT8                     BitSet,
  IN      UINT8                     BitClear,
  IN      UINT32                    TimeOut
  );

EFI_STATUS TcgCountTime (
      IN  UINTN   DelayTime
  )
{
    UINTN           TicksNeeded;
    UINT32          TimerValue, NewTimerValue;
    UINTN           OverFlow;
    UINTN           TheRest, EndValue;
    UINT16          BaseAddr = TimerBaseReg; //porting required

    BaseAddr += ACPI_TIMER_OFFSET;        // *** PORTING NEEDED

    OverFlow = 0;
    TheRest = TicksNeeded = (DelayTime * 358) / 100;
    if (NUM_BITS_IN_ACPI_TIMER < MAX_ACPI_TIMER_BITS)
    {
       OverFlow = TicksNeeded / (1 << NUM_BITS_IN_ACPI_TIMER);
       TheRest = TicksNeeded % (1 << NUM_BITS_IN_ACPI_TIMER);
    }


    // read ACPI Timer
    TimerValue = IoRead32(BaseAddr);

    // need to adjust the values based off of the start time
    EndValue = TheRest + TimerValue;

    // check for overflow on addition.  possibly a problem
    if (EndValue < TimerValue)
    {
       OverFlow++;
    }
    
    // here make sure that EndValue is less than the max value
    //  of the counter
    else if (NUM_BITS_IN_ACPI_TIMER < MAX_ACPI_TIMER_BITS)
    {
       OverFlow += EndValue / (1 << NUM_BITS_IN_ACPI_TIMER);
       EndValue = EndValue % (1 << NUM_BITS_IN_ACPI_TIMER);
    }

    // let the timer wrap around as many times as calculated
    while (OverFlow)
    {
       // read timer amd look to see if the new value read is less than
       //  the current timer value.  if this happens the timer overflowed
       NewTimerValue = IoRead32(BaseAddr);

       if (NewTimerValue < TimerValue)
          OverFlow--;
          TimerValue = NewTimerValue;
     }


     // now wait for the correct number of ticks that need to occur after
     //  all the needed overflows
     while (EndValue > TimerValue){
         NewTimerValue = IoRead32(BaseAddr);

         // check to see if the timer overflowed.  if it did then
         //  the time has elapsed. Because EndValue should be greater than TimerValue
         if (NewTimerValue < TimerValue)
           break;

          TimerValue = NewTimerValue;
      }

      return EFI_SUCCESS;
  }





//**********************************************************************
//<AMI_PHDR_START>
//
// Procedure:   FixedDelay
//
// Description: This routine delays for specified number of micro seconds
//
// Input:       IN UINT32  dCount      Amount of delay (count in 20microsec)
//
// Output:      NONE
//
// Modified:
//
// Referrals:   Div64, CountTime
//
// Notes:       
//<AMI_PHDR_END>
//**********************************************************************

void
__stdcall
FixedDelay(UINT32 dCount)
{
  UINT32  TickPeriod  = 0x03;
  UINTN  MicroSDelay = DELAY_AMOUNT;

  while(dCount) {
      TcgCountTime(MicroSDelay);  
      dCount--;
  }
}


static
UINT8
__stdcall
I2CCheckAccessBit (
  IN      volatile UINT8   *Sts,
  IN      UINT8             Bit,
  IN      UINT32             Timeout    
  )
/*++
Routine Description:
  Function to check bits in TPM access register
Arguments:
  *Sts      - A Pointer to Status register
  Bit       - Bit position
  Timeout    - Timeout amount to wait till the specified bit
Returns:
  EFI_Status
--*/
{
  UINT32  AccessCount = Timeout;            
  UINT8   dataByte;
  if(!( TpmReadByte((UINT32) Sts) & TPM_STS_VALID))
    return 0;

  do {

     FixedDelay((UINT32)Wait);  
    if(( (dataByte=TpmReadByte((UINT32)Sts)) & Bit)){ return dataByte & Bit; }
     AccessCount--;
  } while (AccessCount);

  return 0;    
}



static
UINT8
__stdcall
I2CCheckStsBit (
  IN      volatile UINT8            *Sts,
  IN      UINT8                     Bit
  )
/*++
Routine Description:
  Function to check status a specific TPM status bit 
Arguments:
  *Sts  - A Pointer to Status register
  Bit   - Bit position
Returns:
  EFI_Status
--*/
{
  UINT8 bTpmByte0;
  while (!( (bTpmByte0=TpmReadByte((UINT32)Sts)) & TPM_STS_VALID));
  return bTpmByte0 & Bit;
}

#undef TIS_TIMEOUT_A   
#undef TIS_TIMEOUT_B   
#undef TIS_TIMEOUT_C   
#undef TIS_TIMEOUT_D   
#include "I2CTisLib/TpmCommLib.h"

extern void mStall(UINT32 dMicroSeconds);
#define MicroSecondDelay mStall

EFI_STATUS
EFIAPI
TisPcReadBurstCount (
  IN      TIS_PC_REGISTERS_PTR      TisReg,
     OUT  UINT16                    *BurstCount
  )
{
  UINT32                            WaitTime;
  UINT8                             DataByte0;
  UINT8                             DataByte1;

  if (BurstCount == NULL || TisReg == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  WaitTime = 0;
  do {
    //
    // TIS_PC_REGISTERS_PTR->burstCount is UINT16, but it is not 2bytes aligned,
    // so it needs to use TpmReadByte to read two times
    //
    DataByte0   = TpmReadByte ((UINTN)&TisReg->BurstCount);
    DataByte1   = TpmReadByte ((UINTN)&TisReg->BurstCount + 1);
    *BurstCount = (UINT16)(((UINT16)DataByte1 << 8) + DataByte0);
    if (*BurstCount != 0) {
      return EFI_SUCCESS;
    }
    MicroSecondDelay (30);
    WaitTime += 30;
  } while (WaitTime < TIS_TIMEOUT_D);

  return EFI_TIMEOUT;
}

static
UINT16
__stdcall
I2CReadBurstCount (
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
)
/*++
Routine Description:
  Gets the number of bytes (burstCount) that the TPM can return
  on reads or accept on writes without inserting LPC long wait 
  states on the LPC bus.
  burstCount is TPM_STS_x register bits 8..23
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  0                - Any Errors
  burstCount       - TPM_STS_x.burstCount
--*/  
{
  UINT16   burstCount;
  EFI_STATUS Status;
//  UINT16   dataByte0, dataByte1;
//  UINT64   Deadline = TIS_TIMEOUT_D;

  burstCount = 0;
/*
  do {
    //
    // burstCount is little-endian bit ordering
    //
    FixedDelay((UINT32)Wait);
    Deadline--;
//    burstCount = TpmReg->BurstCount;
    dataByte0   = TpmReadByte ((UINTN)&TpmReg->BurstCount);
    dataByte1   = TpmReadByte ((UINTN)&TpmReg->BurstCount + 1);
    burstCount = (UINT16)((dataByte1 << 8) + dataByte0);

  }while (!burstCount && (Deadline > 0));
*/
  Status = TisPcReadBurstCount((TIS_PC_REGISTERS*)TpmReg, &burstCount);
  if( EFI_ERROR(Status) )
    return 0;
  return burstCount;
}

EFI_STATUS
__stdcall
TisRequestLocality (     //USED by outside routines.
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
  )
/*++
Routine Description:
  Requests TPM locality 0
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  EFI_Status
--*/  
{
  DEBUG((-1, "TisRequestLocality called...\n"));
  if ((TpmReadByte((UINT32)&TpmReg->Access) & TPM_ACC_ACTIVE_LOCALITY)==TPM_ACC_ACTIVE_LOCALITY){
    return EFI_SUCCESS;//EFI_ALREADY_STARTED;
  }
  //TpmReg->Access = TPM_ACC_REQUEST_USE;
  TpmWriteByte((UINT32)&TpmReg->Access, TPM_ACC_REQUEST_USE);
  if (I2CCheckAccessBit(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY, (UINT32)TIS_TIMEOUT_B)) {
    DEBUG((-1, "TisRequestLocality called(success)...\n"));
    return EFI_SUCCESS;
  } else {
    return EFI_NOT_READY;
  }
}



#pragma optimize("",off)
EFI_STATUS
__stdcall
TisReleaseLocality (     //USED by outside routines.
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
  )
/*++
Routine Description:
  Release TPM locality 0
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  EFI_Status
--*/
{
  UINT32    AccessCount=ACCESS_WAITCOUNT;    
  DEBUG((-1, "TisReleaseLocality called...\n"));
  if (!I2CCheckStsBit (&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)) {
    DEBUG((-1, "TisReleaseLocality line %d failed...\n", __LINE__ ));
    return EFI_DEVICE_ERROR;
  }

  //TpmReg->Access = TPM_ACC_ACTIVE_LOCALITY;
  TpmWriteByte((UINTN)&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY);
  if (I2CCheckStsBit(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)) {
    do{/*give locality time to be released*/
     FixedDelay((UINT32)Wait); 
       AccessCount--;
    }while(((I2CCheckStsBit(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)) && AccessCount));
    if(I2CCheckStsBit(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)){
      DEBUG((-1, "TisReleaseLocality line %d failed...\n", __LINE__ ));
      return EFI_DEVICE_ERROR;}
    else{
DEBUG((-1, "TisReleaseLocality called(success)...\n"));
      return EFI_SUCCESS;}
  } else {
    DEBUG((-1, "TisReleaseLocality called(success)...\n"));
    return EFI_SUCCESS;
  }
}
#pragma optimize("",on)

/*
299 static void release_locality(struct tpm_chip *chip, int loc, int force)
300 {
301         u8 buf;
302         if (iic_tpm_read(TPM_ACCESS(loc), &buf, 1) < 0)
303                 return;
304 
305         if (force || (buf & (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID)) ==
306             (TPM_ACCESS_REQUEST_PENDING | TPM_ACCESS_VALID)) {
307                 buf = TPM_ACCESS_ACTIVE_LOCALITY;
308                 iic_tpm_write(TPM_ACCESS(loc), &buf, 1);
309         }
310 }
311 
*/


EFI_STATUS
__stdcall
TisPrepareSendCommand (
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
  )
/*++
Routine Description:
  Helper function to prepare to send a TPM command
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  EFI_Status
--*/
{

 if(( TpmReadByte((UINTN)&TpmReg->Access) & TPM_ACC_ACTIVE_LOCALITY)!=TPM_ACC_ACTIVE_LOCALITY){
    return EFI_NOT_STARTED;
  }

  do {
    //TpmReg->Sts = TPM_STS_READY;
    TpmWriteByte((UINTN)&TpmReg->Sts, TPM_STS_READY);
  } while (!(TpmReadByte((UINTN)&TpmReg->Sts) & TPM_STS_READY));
  return EFI_SUCCESS;
}

EFI_STATUS
__stdcall
I2CTisSendCommand (
  IN      TPM_1_2_REGISTERS_PTR     TpmReg,
  IN      const VOID                *CmdStream,
  IN      UINTN                     Size,
  IN      BOOLEAN                   Last
  )
/*++
Routine Description:
  Function to send TPM command
Arguments:
  TpmReg       - A Pointer to the TPM Register Space
  *CmdStream   - A Pointer to the command stream to be sent to TPM Fifo
  Size         - Size in bytes of the command stream
  Last         - Boolean to signify the last byte?
Returns:
  EFI_Status
--*/
{
  UINT8                             *Ptr;
  UINT16                            burstCount;

  if (Size == 0) {
    return Last ? EFI_INVALID_PARAMETER : EFI_SUCCESS;
  }

  if ((TpmReadByte((UINTN)&TpmReg->Access) & TPM_ACC_ACTIVE_LOCALITY)!=TPM_ACC_ACTIVE_LOCALITY) {
    return EFI_ABORTED;
  }

  Ptr = (UINT8*)CmdStream;

  if (Last) {
    Size--;
  }
DEBUG((DEBUG_ERROR, "in TisSendCommand \n"));
DEBUG((DEBUG_ERROR, "TpmReadByte TpmReg->Sts:[%x]\n", TpmReadByte( (UINT32)&TpmReg->Sts) ));
  while (Size > 0) {
      burstCount = I2CReadBurstCount(TpmReg);
    if (burstCount == 0) {
      // Cannot get the correct burstCount value
      return EFI_TIMEOUT;
    }
DEBUG((DEBUG_ERROR, "\t\t\t Burst Count %x \n", burstCount ));
DEBUG((DEBUG_ERROR, "\t\t\t "));
    for (; burstCount > 0 && Size > 0; burstCount--) {
      DEBUG((DEBUG_ERROR, " %2x ", *Ptr ));
     //*(UINT8*)&TpmReg->DataFifo = *Ptr;
      TpmWriteByte( (UINTN)&TpmReg->DataFifo, *Ptr);

      Ptr++;
      Size--;
    }
   }
//DEBUG((DEBUG_ERROR, "TpmReadByte TpmReg->Sts:[%x]\n", TpmReadByte( (UINT32)&TpmReg->Sts) ));
  if (Last) {
    EFI_STATUS Status;
    /*
    if (!I2CCheckStsBit (&TpmReg->Sts, TPM_STS_EXPECT)) {
      DEBUG((DEBUG_ERROR, "line: %d abort\n", __LINE__ ));
      return EFI_ABORTED;
    }
    */
    //*(UINT8*)&TpmReg->DataFifo = *Ptr;
    TpmWriteByte((UINTN)&TpmReg->DataFifo, *Ptr); 
    /*
    MicroSecondDelay (300);
    if (I2CCheckStsBit (&TpmReg->Sts, TPM_STS_EXPECT)) {
      DEBUG((DEBUG_ERROR, "line: %d abort\n", __LINE__ ));
      return EFI_ABORTED;
    }
    */
      //
  // Ensure the Tpm status STS_EXPECT change from 1 to 0
  //
    Status = TisPcWaitRegisterBits (
             (UINT8*)(&TpmReg->Sts),
             (UINT8) TIS_PC_VALID,
             TIS_PC_STS_EXPECT,
             TIS_TIMEOUT_C
             );
    if(EFI_ERROR(Status)) {
      DEBUG((DEBUG_ERROR, "line: %d abort\n", __LINE__ ));
      return EFI_ABORTED; 
    }

    //TpmReg->Sts = TPM_STS_GO;
    TpmWriteByte((UINTN)&TpmReg->Sts, TPM_STS_GO);
  }
  DEBUG((DEBUG_ERROR, "\t\t\t  Success \n"));
  return EFI_SUCCESS;
}




EFI_STATUS
__stdcall
I2CTisWaitForResponse (
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
  )
/*++
Routine Description:
  Waits till TPM result is available
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  EFI_Status
--*/
{
  if((TpmReadByte( (UINTN)&TpmReg->Access) & TPM_ACC_ACTIVE_LOCALITY)!=TPM_ACC_ACTIVE_LOCALITY){
    return EFI_NOT_STARTED;
  }

  while (!I2CCheckStsBit (&TpmReg->Sts, TPM_STS_DATA_AVAILABLE));
  return EFI_SUCCESS;
}




EFI_STATUS
__stdcall
I2CTisReceiveResponse (
  IN      TPM_1_2_REGISTERS_PTR     TpmReg,
  OUT     VOID                      *Buffer,
  OUT     UINTN                     *Size
  )
/*++
Routine Description:
  Function to recieve TPM command results
Arguments:
  TpmReg       - A Pointer to the TPM Register Space
  *Buffer      - A Pointer to buffer for recieving result data
  Size         - buffer size
Returns:
  EFI_Status
--*/
{
  UINT8                             *Ptr, *BufEnd;
  UINT16                             burstCount;

  if((TpmReadByte((UINTN)&TpmReg->Access) & TPM_ACC_ACTIVE_LOCALITY)!=TPM_ACC_ACTIVE_LOCALITY) {
      return EFI_ABORTED;
  }

  Ptr = (UINT8*)Buffer;
  BufEnd = Ptr + *Size;
  while (Ptr < BufEnd &&
         I2CCheckStsBit (&TpmReg->Sts, TPM_STS_DATA_AVAILABLE)) {
      
        burstCount = I2CReadBurstCount(TpmReg);
          if (burstCount == 0) {
              return EFI_TIMEOUT;
        }            
        while(burstCount && Ptr < BufEnd  && I2CCheckStsBit (&TpmReg->Sts, TPM_STS_DATA_AVAILABLE )){
            //*Ptr++ = *(UINT8*)&TpmReg->DataFifo;
			      *Ptr++ = TpmReadByte((UINTN)&TpmReg->DataFifo);
            burstCount--;    }
  }

  *Size -= BufEnd - Ptr;
  if (I2CCheckStsBit (&TpmReg->Sts, TPM_STS_DATA_AVAILABLE)) {
    return EFI_BUFFER_TOO_SMALL;
  } else {
    return EFI_SUCCESS;
  }
}



VOID
__stdcall
I2CTisResendResponse (
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
  )
/*++
Routine Description:
  Sets Bit to resend TPM command
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  EFI_Status
--*/
{
  //TpmReg->Sts = TPM_STS_RESPONSE_RETRY;
  TpmWriteByte((UINTN)&TpmReg->Sts, TPM_STS_RESPONSE_RETRY);
}


#define PCI_EXPRESS_BASE_ADDRESS ((VOID *) (UINTN) PCIEX_BASE_ADDRESS)

#define MmPciAddress( Segment, Bus, Device, Function, Register ) \
  ( (UINTN)PCI_EXPRESS_BASE_ADDRESS + \
    (UINTN)(Bus << 20) + \
    (UINTN)(Device << 15) + \
    (UINTN)(Function << 12) + \
    (UINTN)(Register) \
  )

extern VOID DumpStructData(UINT8* bData, UINTN nSize);

//
// Serial IO Controllers Private Registers
//
#define R_PCH_LP_SERIAL_IO_PPR_CLK                      0x800
#define B_PCH_LP_SERIAL_IO_PPR_CLK_EN                   BIT0
#define B_PCH_LP_SERIAL_IO_PPR_CLK_UPDATE               BIT31
#define V_PCH_LP_SERIAL_IO_PPR_CLK_M_DIV                0x25A
#define V_PCH_LP_SERIAL_IO_PPR_CLK_N_DIV                0x7FFF

#define R_PCH_LP_SERIAL_IO_PPR_RST                      0x804
#define B_PCH_LP_SERIAL_IO_PPR_RST_APB                  BIT0
#define B_PCH_LP_SERIAL_IO_PPR_RST_FUNC                 BIT1
#define R_PCH_LP_SERIAL_IO_PPR_GEN                      0x808
#define B_PCH_LP_SERIAL_IO_PPR_GEN_LTR_MODE             BIT2
#define B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL       BIT3
#define R_PCH_LP_SERIAL_IO_PPR_AUTO_LTR                 0x814


EFI_STATUS
__stdcall
IsTpmPresent (     //USED by outside routines.
  IN      TPM_1_2_REGISTERS_PTR     TpmReg
  )
{
//return EFI_NOT_FOUND;

if(1){
  volatile UINT32* mI2c0;
  /*
  mI2c0 = (volatile UINT32*)0xFE104000;
  DEBUG((DEBUG_ERROR, "data at 0xFE104000 is %x\n", *mI2c0 ));

  mI2c0 = (volatile UINT32*)0xFE104010;
  DEBUG((DEBUG_ERROR, "data at 0xFE104010 is %x\n", *mI2c0 ));
  
  if( *mI2c0 != 0xFFFFFFFF && *mI2c0 != 0)
    DumpStructData((UINT8*)(UINT32)(*mI2c0), 0x100);
  mI2c0 = (volatile UINT32*)0xFE104014;
  DEBUG((DEBUG_ERROR, "data at 0xFE104014 is %x\n", *mI2c0 ));
  */

  mI2c0 = (volatile UINT32*)MmPciAddress(0, 0, 24, 1, 0);
  DEBUG((DEBUG_ERROR, "data at %x is %x\n", mI2c0, *mI2c0 ));
  mI2c0 = (volatile UINT32*)MmPciAddress(0, 0, 24, 1, 0x10);
  DEBUG((DEBUG_ERROR, "data at %x is %x\n", mI2c0, *mI2c0 ));
  if( *mI2c0 != 0xFFFFFFFF && *mI2c0 != 0)
    DumpStructData((UINT8*)(UINT32)(*mI2c0), 0x100);
  else {
      *mI2c0 = 0xD091A000;  //0xFE103000;
      *(volatile UINT8*)MmPciAddress(0, 0, 24, 1, 4) = 0x07;

      mI2c0 = (volatile UINT32*)(0xD091A000+R_PCH_LP_SERIAL_IO_PPR_RST);
      
//      if( (((UINT32)*mI2c0)&((B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB))) 
//          != (B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB) ) 
      {

      mI2c0 = (volatile UINT32*)0xD091A000;
      DEBUG((-1, "PCI data... \n"));
      DumpStructData((UINT8*)MmPciAddress(0, 0, 24, 1, 0), 0x100);
      DEBUG((-1, "BAR0 data... \n"));
      DumpStructData((UINT8*)(UINT32)(mI2c0), 0x100);

      DEBUG((-1, "BAR0 data.+800h.%x. \n", 0xD091A800));
      DumpStructData((UINT8*)(UINT32)(0xD091A000+0x800), 0x100);     
      
      /*
      /// PCH BIOS Spec Rev 0.7.0 Section 23.3 Serial IO LTR Programming
      /// Step 1: Program BAR0 + 808h[2] = 0b
      /// Step 2: Program BAR0 + 804h[1:0] = 00b
      /// Step 3: Program BAR0 + 804h[1:0] = 11b
      ///
      MmioAndThenOr32WithScript (SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_GEN, (UINT32)(~B_PCH_LP_SERIAL_IO_PPR_GEN_LTR_MODE), 0x0);
      MmioAndThenOr32WithScript (SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_RST, (UINT32)(~(B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB)), 0);
      MmioAndThenOr32WithScript (SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_RST, 0xFFFFFFFF, (UINT32)(B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB));
  
      ///
      /// Step 4
      /// Program BAR0 + 814h with LTR value for each SerialIo controller
      ///
      MmioAndThenOr32WithScript(SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_AUTO_LTR, 0, 0);
    }
  
    ///
    /// PCH BIOS Spec Rev 0.7.0 Section 23.7 Serial IO Power Management Programming
    /// Step 4
    /// Program IO Voltage Select for I2C0 & I2C1 as per platform policy
    ///
    if(i == PCI_FUNCTION_NUMBER_PCH_LP_SERIAL_IO_I2C0) {
      if (PchPlatformPolicy->SerialIoConfig.I2c0VoltageSelect == PchSerialIoIs33V) {
        MmioAndThenOr32WithScript(SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_GEN, (UINT32)~(B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL), 0);
      } else {
        MmioAndThenOr32WithScript(SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_GEN, 0xFFFFFFFF, B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL);
      }
    }
      */
      mI2c0 = (volatile UINT32*)(0xD091A000+R_PCH_LP_SERIAL_IO_PPR_GEN);
      *mI2c0 &= (UINT32)(~B_PCH_LP_SERIAL_IO_PPR_GEN_LTR_MODE);

//      mI2c0 = (volatile UINT32*)(0xD091A000+R_PCH_LP_SERIAL_IO_PPR_RST);
//      *mI2c0 &= (UINT32)(~(B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB));
      
      mI2c0 = (volatile UINT32*)(0xD091A000+R_PCH_LP_SERIAL_IO_PPR_RST);
      *mI2c0 |= (UINT32)(UINT32)(B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB);

/*     
      MmioAndThenOr32WithScript (SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_GEN, (UINT32)(~B_PCH_LP_SERIAL_IO_PPR_GEN_LTR_MODE), 0x0);
      MmioAndThenOr32WithScript (SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_RST, (UINT32)(~(B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB)), 0);
      MmioAndThenOr32WithScript (SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_RST, 0xFFFFFFFF, (UINT32)(B_PCH_LP_SERIAL_IO_PPR_RST_FUNC | B_PCH_LP_SERIAL_IO_PPR_RST_APB));
*/  
      ///
      /// Step 4
      /// Program BAR0 + 814h with LTR value for each SerialIo controller
      ///
      //MmioAndThenOr32WithScript(SerialIoDevice[i].Bar0 + R_PCH_LP_SERIAL_IO_PPR_AUTO_LTR, 0, 0);
      /*
      mI2c0 = (volatile UINT32*)(0xD091A000+R_PCH_LP_SERIAL_IO_PPR_AUTO_LTR);
      *mI2c0 &= 0;
      */

      mI2c0 = (volatile UINT32*)(0xD091A000+0x808);

/* no B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL for bay trial
#if _1_V_8_VOLTAGE_ON_I2C
      //1.8V
      *mI2c0 &= 0xFFFFFFFF;
      *mI2c0 |= B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL;   //B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL
#else
      *mI2c0 &= (UINT32)~(B_PCH_LP_SERIAL_IO_PPR_GEN_IO_VOLTAGE_SEL);
      *mI2c0 |= 0x00;   // 
#endif
*/
      //
      // Activate Clocks
      //
      mI2c0 = (volatile UINT32*)(0xD091A000+R_PCH_LP_SERIAL_IO_PPR_CLK);
      *mI2c0 = 0x80020003;
      
      mI2c0 = (volatile UINT32*)0xD091A000;
      DEBUG((-1, "after PCI data... \n"));
      DumpStructData((UINT8*)MmPciAddress(0, 0, 24, 1, 0), 0x100);
      DEBUG((-1, "after BAR0 data... \n"));
      DumpStructData((UINT8*)(UINT32)(mI2c0), 0x100);

      DEBUG((-1, "after BAR0 data.+800h.%x. \n", 0xD091A800));
      DumpStructData((UINT8*)(UINT32)(0xD091A000+0x800), 0x100);    
      } 
  }

/*
UINT16 I2CGPIO[]= {
  
   19.1.6  I2C0
   I2C0_SDA-OD-O -    write 0x2003CC81 to IOBASE + 0x0210
   I2C0_SCL-OD-O -    write 0x2003CC81 to IOBASE + 0x0200
  0x0210,
  0x0200,
*/
  //IO_BASE_ADDRESS+I2CGPIO[Index], 0x2003CC81
  mI2c0 = (volatile UINT32*)(IO_BASE_ADDRESS+0x0210);
  *mI2c0 = 0x2003CC81;
  mI2c0 = (volatile UINT32*)(IO_BASE_ADDRESS+0x0200);
  *mI2c0 = 0x2003CC81;

  mI2c0 = (volatile UINT32*)MmPciAddress(0, 0, 24, 1, 0x14);
  DEBUG((DEBUG_ERROR, "data at %x is %x\n", mI2c0, *mI2c0 ));
}
//DEBUG((DEBUG_ERROR, "delay one second..." ));
//MicroSecondDelay(3000*1000);

DEBUG((-1, "TPM VID-DID is [%x]\n", TpmReadByte(0x06) ));
DEBUG((-1, "TPM VID-DID is [%x]\n", TpmReadByte(0x07) ));
DEBUG((-1, "TPM VID-DID is [%x]\n", TpmReadByte(0x08) ));
DEBUG((-1, "TPM VID-DID is [%x]\n", TpmReadByte(0x09) ));

  if (TpmReadByte((UINTN)&TpmReg->Access) == 0xff) {
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}

extern EFI_STATUS  
I2CWriteByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN OUT VOID                       *Buffer
  );

extern EFI_STATUS  
I2CReadByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN OUT VOID                       *Buffer
  );

extern EFI_STATUS
EFIAPI
I2CWriteMultipleByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN UINTN                          *Length,
  IN OUT    VOID                    *Buffer
  );

/*
#undef TIS_TIMEOUT_A   
#undef TIS_TIMEOUT_B   
#undef TIS_TIMEOUT_C   
#undef TIS_TIMEOUT_D   
#include "I2CTisLib/TpmCommLib.h"
*/
EFI_STATUS
EFIAPI
TisPcWaitRegisterBits (
  IN      UINT8                     *Register,
  IN      UINT8                     BitSet,
  IN      UINT8                     BitClear,
  IN      UINT32                    TimeOut
  )
{
  UINT8                             RegRead;
  UINT32                            WaitTime;

  for (WaitTime = 0; WaitTime < TimeOut; WaitTime += 30){
    RegRead = TpmReadByte ((UINTN)Register);
//DEBUG((-1, "TPM read byte is [%x]\n", RegRead ));
    if ((RegRead & BitSet) == BitSet && (RegRead & BitClear) == 0)
      return EFI_SUCCESS;
    MicroSecondDelay (30);
  }
  return EFI_TIMEOUT;
}


EFI_STATUS
EFIAPI
TisPcRequestUseTpm (
  IN      TIS_PC_REGISTERS_PTR      TisReg
  )
{
  EFI_STATUS                        Status;
  
  if (TisReg == NULL) {
    return EFI_INVALID_PARAMETER;
  }
/*  
  if (!TisPcPresenceCheck (TisReg)) {
    return EFI_NOT_FOUND;
  }
*/
  TpmWriteByte ((UINTN)&TisReg->Access, TIS_PC_ACC_RQUUSE);
  Status = TisPcWaitRegisterBits (
             &TisReg->Access,
             (UINT8)(TIS_PC_ACC_ACTIVE |TIS_PC_VALID),
             0,
             TIS_TIMEOUT_D
             );
  return Status;
}

#if 1

static
UINT8
__stdcall
CheckStsBitI2CTpm (
  IN      volatile UINT8            *Sts,
  IN      UINT8                     Bit
  )
/*++
Routine Description:
  Function to check status a specific TPM status bit 
Arguments:
  *Sts  - A Pointer to Status register
  Bit   - Bit position
Returns:
  EFI_Status
--*/
{
  //while (!(*Sts & TPM_STS_VALID));

  UINT8                             RegRead;
  //UINT32                            WaitTime;

  do {
      RegRead = TpmReadByte ((UINTN)Sts);
      MicroSecondDelay (30);
  } while( !(RegRead&TPM_STS_VALID) );

  /*
  for (WaitTime = 0; WaitTime < TimeOut; WaitTime += 30){
    RegRead = TpmReadByte ((UINTN)Sts);
    if ( (RegRead & TPM_STS_VALID) == TPM_STS_VALID )
      return RegRead & Bit;
    MicroSecondDelay (30);
  }
  */
  return RegRead & Bit;
}

#pragma optimize("",off)
EFI_STATUS
__stdcall
TisReleaseLocalityTpm (
  IN      TIS_PC_REGISTERS_PTR     TpmReg
  )
/*++
Routine Description:
  Release TPM locality 0
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
Returns:
  EFI_Status
--*/
{
  UINT32    AccessCount=ACCESS_WAITCOUNT;    
  if (!CheckStsBitI2CTpm (&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)) {
    return EFI_DEVICE_ERROR;
  }

  //TpmReg->Access = TPM_ACC_ACTIVE_LOCALITY;
  TpmWriteByte ((UINTN)&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY);

  if (CheckStsBitI2CTpm(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)) {
    do{/*give locality time to be released*/
     FixedDelay((UINT32)Wait); 
       AccessCount--;
    }while(((CheckStsBitI2CTpm(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)) && AccessCount));
    if(CheckStsBitI2CTpm(&TpmReg->Access, TPM_ACC_ACTIVE_LOCALITY)){return EFI_DEVICE_ERROR;}
    else{return EFI_SUCCESS;}
  } else {
    return EFI_SUCCESS;
  }
}
#pragma optimize("",on)
#endif

EFI_STATUS
EFIAPI
TisPcPrepareCommand (
  IN      TIS_PC_REGISTERS_PTR      TisReg
  )
{
  EFI_STATUS                        Status;

  if (TisReg == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  TpmWriteByte ((UINTN)&TisReg->Status, TIS_PC_STS_READY);
  Status = TisPcWaitRegisterBits (
             &TisReg->Status,
             TIS_PC_STS_READY,
             0,
             TIS_TIMEOUT_B
             );
  return Status;
}


EFI_STATUS
TisPcSend (
  IN     TIS_PC_REGISTERS_PTR       TisReg,
  IN     UINT8                      *TpmBuffer,
  IN     UINT32                     DataLength
  )
{
  UINT16                            BurstCount;
  UINT32                            Index;
  EFI_STATUS                        Status;

  Status = TisPcPrepareCommand (TisReg);
  if (EFI_ERROR (Status)){
    DEBUG ((DEBUG_ERROR, "The Tpm not ready!\n"));
    return Status;
  }
  Index = 0;
  while (Index < DataLength) {
    Status = TisPcReadBurstCount (TisReg, &BurstCount);
    DEBUG ((DEBUG_ERROR, "BurstCount is %x\n", BurstCount));
    if (EFI_ERROR (Status)) {
      return EFI_TIMEOUT;
    }
    for (; BurstCount > 0 && Index < DataLength; BurstCount--) {
      DEBUG((DEBUG_ERROR, " %2x ", *(TpmBuffer + Index) ));
      TpmWriteByte ((UINTN) &TisReg->DataFifo, *(TpmBuffer + Index));
      Index++;
    }
  }
  //
  // Ensure the Tpm status STS_EXPECT change from 1 to 0
  //
  Status = TisPcWaitRegisterBits (
             &TisReg->Status,
             (UINT8) TIS_PC_VALID,
             TIS_PC_STS_EXPECT,
             TIS_TIMEOUT_C
             );

  //TpmReg->Sts = TPM_STS_GO;
  TpmWriteByte((UINTN)&TisReg->Status, TPM_STS_GO);
  DEBUG((DEBUG_ERROR, "\t\t\t  Success \n"));

  return Status;
}


EFI_STATUS
__stdcall
TpmLibPassThrough (      //USED by outside routines.
  IN      TPM_1_2_REGISTERS_PTR     TpmReg,
  IN      UINTN                     NoInputBuffers,
  IN      TPM_TRANSMIT_BUFFER       *InputBuffers,
  IN      UINTN                     NoOutputBuffers,
  IN OUT  TPM_TRANSMIT_BUFFER       *OutputBuffers
  )
/*++
Routine Description:
  Higher level function to send a recieve commands to the TPM
Arguments:
  TpmReg           - A Pointer to the TPM Register Space
  NoInputBuffers   - Number count of Input buffers
  *InputBuffers    - Pointer to InputBuffers[0]
  NoOutputBuffers  - Number count of Output buffers
  *OutputBuffers   - Pointer to OutputBuffers[0]
Returns:
  EFI_Status
--*/
{
  EFI_STATUS                        Status;
  UINTN                             i;
  /*
DEBUG((DEBUG_ERROR, "TpmLibPassThrough called, NoInputBuffers %d\n", NoInputBuffers));

DEBUG((DEBUG_ERROR, "delay one second..." ));
mStall(1000*1000);
DEBUG((DEBUG_ERROR, "  time out\n" ));
*/
{

DEBUG((-1, "I2C protocol tests.... , NoInputBuffers %d\n", NoInputBuffers));
if(0){
      UINT8                   Data[10];
      UINTN                   nCmdLength;
      UINTN                   TpmAddress;
      EFI_I2C_DEVICE_ADDRESS  I2CDeviceAddr;
      EFI_I2C_ADDR_MODE       I2CAddrMode;

      //
      // Write addresss to TPM.
      //
      TpmAddress = 0x00;
      Data[0] = (UINT8)TpmAddress;
      I2CDeviceAddr.I2CDeviceAddress = 0xA0>>1;
      I2CAddrMode = EfiI2CSevenBitAddrMode;
      nCmdLength = 1;
#if 1
/*  //Byte read/write
*/
      Status = I2CWriteByte (
                          NULL,
                          I2CDeviceAddr,
                          I2CAddrMode,
                          &Data
                          );

      if (EFI_ERROR(Status)) {
        DEBUG ((EFI_D_INFO, "I2CWriteByte(): write to TPM address %0x failed (%r)\n", TpmAddress, Status));
      }

      //
      // Read data from TPM.
      //
      Data[0] = (UINT8)TpmAddress;
      Status = I2CReadByte (
                          NULL,
                          I2CDeviceAddr,
                          I2CAddrMode,
                          &Data
                          );      
      DEBUG((-1, "Status is %r, Data at 0x00 is %x\n", Status, Data[0]));
#endif
      TpmAddress = 0x00;
      Data[0] = (UINT8)TpmAddress;
      I2CDeviceAddr.I2CDeviceAddress = 0xA8>>1;
      I2CAddrMode = EfiI2CSevenBitAddrMode;      
/*
  //Word read/write
*/
      nCmdLength = 2;
      Data[0] = 0;
      Data[1] = 0;
      Data[2] = 0;
      Status = I2CWriteMultipleByte (
                          NULL,
                          I2CDeviceAddr,
                          I2CAddrMode,
                          &nCmdLength,
                          &Data
                          );


      if (EFI_ERROR(Status)) {
        DEBUG ((EFI_D_INFO, "I2CWriteMultipleByte(): write to TPM address %0x failed (%r)\n", TpmAddress, Status));
      }
      //no idea about this. mI2CPrevReadTransfer = FALSE;
//  I2CWriteByte,
//  I2CReadByte,
      //
      // Read data from TPM.
      //
      Data[0] = (UINT8)TpmAddress;
      Status = I2CReadByte (
                          NULL,
                          I2CDeviceAddr,
                          I2CAddrMode,
                          &Data
                          );
      if (EFI_ERROR(Status)) {
        DEBUG ((EFI_D_INFO, "TpmReadByte(): write to TPM address %0x failed (%r)\n", TpmAddress, Status));
      }
      DEBUG((-1, "Data at 0x00 is %x\n", Data[0]));

      Status = I2CReadByte (
                          NULL,
                          I2CDeviceAddr,
                          I2CAddrMode,
                          &Data
                          );
      if (EFI_ERROR(Status)) {
        DEBUG ((EFI_D_INFO, "TpmReadByte(): write to TPM address %0x failed (%r)\n", TpmAddress, Status));
      }
      DEBUG((-1, "Data at 0x01 is %x\n", Data[0]));

}
DEBUG((-1, "I2C protocol tests ends....\n"));

}
/*
TisPcRequestUseTpm((TIS_PC_REGISTERS_PTR)TpmReg);
Status = TisReleaseLocalityTpm((TIS_PC_REGISTERS_PTR)TpmReg);
DEBUG((-1, "TisReleaseLocalityTpm status: %r....\n", Status ));
*/
DEBUG((-1, "Before processing NoInputBuffers %d\n", NoInputBuffers));
  if (NoInputBuffers == 0 || InputBuffers->Size < sizeof (TPM_1_2_CMD_HEADER)) {
    return EFI_INVALID_PARAMETER;
  }

  do {
    NoInputBuffers--; //one buffer index was Stolen
  } while (InputBuffers[NoInputBuffers].Size == 0 && NoInputBuffers > 0);

  if (InputBuffers[NoInputBuffers].Size == 0) {
    return EFI_INVALID_PARAMETER;
  }
DEBUG((-1, "After processing NoInputBuffers %d\n", NoInputBuffers));
  Status = TisPrepareSendCommand (TpmReg);
DEBUG((-1, "TisPrepareSendCommand %r\n", Status));

  for (i = 0; !EFI_ERROR (Status) && i < NoInputBuffers; i++) {
    DEBUG((DEBUG_ERROR, "(000) Input buffer idx: %d ", i));
    Status = I2CTisSendCommand (
      TpmReg,
      InputBuffers[i].Buffer,
      InputBuffers[i].Size,
      FALSE
      );
  }

  if (!EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "(111) Input buffer idx: %d ", i));
    //Status = TisPcSend( (TIS_PC_REGISTERS_PTR)TpmReg, InputBuffers[i].Buffer, (UINT32)(InputBuffers[i].Size)) ;
    
    Status = I2CTisSendCommand (
      TpmReg,
      InputBuffers[i].Buffer,
      InputBuffers[i].Size,
      TRUE
      );
  }

  if (!EFI_ERROR (Status)) {
    DEBUG((DEBUG_ERROR, "Calling TisWaitForResponse \n"));
    Status = I2CTisWaitForResponse (TpmReg);
  }

  if (EFI_ERROR (Status)) {
    goto Exit;
  }

  Status = EFI_BUFFER_TOO_SMALL;
  for (i = 0; Status == EFI_BUFFER_TOO_SMALL && i < NoOutputBuffers; i++) {
    DEBUG((DEBUG_ERROR, "TisReceiveResponse idx: %d \n", i));
    Status = I2CTisReceiveResponse (
      TpmReg,
      OutputBuffers[i].Buffer,
      &OutputBuffers[i].Size
      );
    {
      UINTN n;
      UINT8*  dptr = (UINT8*)(OutputBuffers[i].Buffer);
      for(n=0;n<OutputBuffers[i].Size;n++)
        DEBUG((DEBUG_ERROR, "OB:: %x \n", dptr[n] ));
    }
  }

Exit:
  TisCompleteCommand (TpmReg);
  return Status;
}
