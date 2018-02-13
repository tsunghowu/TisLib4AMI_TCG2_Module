/** @file
  TPM Generic Write/Read Access routines.

Copyright (c) 2013 Intel Corporation.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.
* Neither the name of Intel Corporation nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**/

#include "..\..\I2CLib\CommonHeader.h"
#include "TpmAccess.h"

//
// Global variable to cache pointer to I2C protocol.
//
//<CHW+>  EFI_I2C_HC_PROTOCOL      *mI2Cbus = NULL;
#define GUIDForPrevReadTransfer  {0xfa63fc49, 0x8c0d, 0x4128, 0x91, 0xd1, 0xfc, 0x85, 0xbd, 0x26, 0x15, 0xcc}
EFI_GUID  mGuid2ICPrevReadTransfer = GUIDForPrevReadTransfer;

//
// Global variable to indicate if TPM I2C Read Transfer has previously occurred.
//
//<CHW+> BOOLEAN mI2CPrevReadTransfer = FALSE;

extern 
EFI_STATUS  
I2CWriteByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN OUT VOID                       *Buffer
  );

extern
EFI_STATUS  
I2CReadByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN OUT VOID                       *Buffer
  );

extern
EFI_STATUS  
I2CWriteMultipleByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN UINTN                          *Length,
  IN OUT VOID                       *Buffer
  );

extern
EFI_STATUS  
I2CReadMultipleByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN UINTN                          *WriteLength,
  IN UINTN                          *ReadLength,
  IN OUT VOID                       *Buffer
  );

extern 
void
mStall(UINT32 dMicroSeconds);

#define MicroSecondDelay mStall

#pragma pack(push, 1)
typedef struct {
  EFI_HOB_GUID_TYPE   EfiHobGuidType;
  BOOLEAN mI2CPrevReadTransfer;
} EFI_HOB_I2C_PREV_READ_TRANSFER;
#pragma pack(pop)

/**
  Maps a standard TCG MMIO address (offset to Locality 0)
  to Infineon SLB9645 TPM I2C Address.

  @param  MMIO Address to map.

  @return Mapped Infineon SLB9645 TPM I2C Address.

**/
UINTN
MapMmioAddr2InfI2CAddr (
  IN  UINTN  Address
  )
{
  UINTN  TpmAddress;

  switch (Address) {
    case (TPM_MMIO_BASE_ADDRESS+TPM_ACCESS_0_ADDRESS_DEFAULT):
      TpmAddress = INFINEON_TPM_ACCESS_0_ADDRESS_DEFAULT;
      break;
    case (TPM_MMIO_BASE_ADDRESS+TPM_STS_0_ADDRESS_DEFAULT):
      TpmAddress = INFINEON_TPM_STS_0_ADDRESS_DEFAULT;
      break;
    case (TPM_MMIO_BASE_ADDRESS+TPM_BURST0_COUNT_0_DEFAULT):
      TpmAddress = INFINEON_TPM_BURST0_COUNT_0_DEFAULT;
      break;
    case (TPM_MMIO_BASE_ADDRESS+TPM_BURST1_COUNT_0_DEFAULT):
      TpmAddress = INFINEON_TPM_BURST1_COUNT_0_DEFAULT;
      break;
    case (TPM_MMIO_BASE_ADDRESS+TPM_DATA_FIFO_0_DEFAULT):
      TpmAddress = INFINEON_TPM_DATA_FIFO_0_ADDRESS_DEFAULT;
      break;
    default:
      TpmAddress = Address;
  }

  // Assert if TPM Address is not within Infineon SLB9645 Locality 0 address range.
  ASSERT (TpmAddress >= INFINEON_TPM_ACCESS_0_ADDRESS_DEFAULT);
  ASSERT (TpmAddress <= INFINEON_TPM_DID_VID_0_DEFAULT);

  return TpmAddress;
}


/**
  Writes single byte data to TPM specified by MMIO address.

  Write access to TPM is MMIO or I2C (based on platform type).

  @param  Address The register to write.
  @param  Data    The data to write to the register.

**/
VOID
TpmWriteByte (
  IN  UINTN  Address,
  IN  UINT8  Data
  )
{
  UINTN                   TpmAddress;
  UINTN                   WriteLength;
  UINT8                   WriteData[2];

//<CHW+>  EFI_PLATFORM_INFO       *PlatformInfo;

  EFI_I2C_DEVICE_ADDRESS  I2CDeviceAddr;
  EFI_I2C_ADDR_MODE       I2CAddrMode;

  EFI_STATUS              Status;
  BOOLEAN *pI2CPrevReadTransfer;
  EFI_HOB_GUID_TYPE       *GuidHob;

//DEBUG((-1, "**** TpmWriteByte called\n"));
  //
  // Get platform type from platform info hob.
  //
  GuidHob      = GetFirstGuidHob (&mGuid2ICPrevReadTransfer);
  if(GuidHob != NULL )
    pI2CPrevReadTransfer = GET_GUID_HOB_DATA (GuidHob);
  else {
#ifdef ADLINK_PEI_COMPILE     
    const EFI_PEI_SERVICES **PeiServices;
    EFI_HOB_I2C_PREV_READ_TRANSFER *pI2cPrevHob;
    PeiServices = GetPeiServicesTablePointer ();

    Status = (*PeiServices)->CreateHob (PeiServices, EFI_HOB_TYPE_GUID_EXTENSION, 
                            sizeof(BOOLEAN) + sizeof (EFI_HOB_GUID_TYPE), (VOID**)&pI2cPrevHob);

    if (EFI_ERROR(Status)) return;
    //MemCpy (&(pI2cPrevHob->EfiHobGuidType.Name), &mGuid2ICPrevReadTransfer, sizeof (EFI_GUID));    
    pI2cPrevHob->EfiHobGuidType.Name = mGuid2ICPrevReadTransfer;

    pI2cPrevHob->mI2CPrevReadTransfer = FALSE;
    pI2CPrevReadTransfer = &(pI2cPrevHob->mI2CPrevReadTransfer);
#else 

#endif    
  }
//DEBUG((-1, "TpmWriteByte called: pI2CPrevReadTransfer is %s\n", *pI2CPrevReadTransfer?"TRUE":"FALSE"));

  //<CHW+>ASSERT (PlatformInfo != NULL);

  //
  // Only Crosshill platform supports TPM via I2C bus.
  // otherwise do nothing.
  //
  //<CHW+>if (PlatformInfo->Type == (EFI_PLATFORM_TYPE) CrossHill) {
    //
    // Locate I2C protocol for TPM I2C access.
    //
     //<CHW+>Status = gBS->LocateProtocol (&gEfiI2CHcProtocolGuid, NULL, (VOID**) &mI2Cbus);
     //<CHW+>if (EFI_ERROR(Status)) {
     //<CHW+>  DEBUG ((EFI_D_ERROR, "TpmWriteByte(): could not locate I2CHcProtocol\n"));
     //<CHW+>}
    //<CHW+> ASSERT_EFI_ERROR (Status);

    //
    // Setup I2C Slave device address and address mode (7-bit).
    //
    I2CDeviceAddr.I2CDeviceAddress = TPM_I2C_SLAVE_DEVICE_ADDRESS;
    I2CAddrMode = EfiI2CSevenBitAddrMode;

    //
    // As recommended by Infineon (SLB9645 I2C Communication protocol application
    // note revision 1.0) wait 250 microseconds between a read and a write transfer.
    //
    //if (*pI2CPrevReadTransfer) {
      MicroSecondDelay (GUARD_TIME);
    //}

    //
    // Map MMIO address to Infineon I2C address.
    //
    TpmAddress = MapMmioAddr2InfI2CAddr (Address);

    //
    // Write to TPM register.
    //
    WriteLength = 2;
    WriteData[0] = (UINT8)TpmAddress;
    WriteData[1] = Data;

    Status = I2CWriteMultipleByte (
                        NULL,
                        I2CDeviceAddr,
                        I2CAddrMode,
                        &WriteLength,
                        &WriteData
                        );

    if (EFI_ERROR(Status)) {
      DEBUG ((EFI_D_ERROR, "TpmWriteByte(): I2C Write to TPM address %0x failed (%r)\n", TpmAddress, Status));
      ASSERT (FALSE);  // Writes to TPM should always succeed.
    }

    *pI2CPrevReadTransfer = FALSE;
  //<CHW+>}
}


/**
  Reads single byte data from TPM specified by MMIO address.

  Read access to TPM is via MMIO or I2C (based on platform type).

  Due to stability issues when using I2C combined write/read transfers (with
  RESTART) to TPM (specifically read from status register), a single write is 
  performed followed by single read (with STOP condition in between).

  @param  Address of the MMIO mapped register to read.

  @return The value read.

**/
UINT8
EFIAPI
TpmReadByte (
  IN  UINTN  Address
  )
{
  UINTN                   TpmAddress;
  UINT8                   Data[1];
  UINT8                   ReadData;
  UINT8                   ReadCount;

//<CHW+>   EFI_PLATFORM_INFO       *PlatformInfo;
//<CHW+>   EFI_HOB_GUID_TYPE       *GuidHob;

  EFI_I2C_DEVICE_ADDRESS  I2CDeviceAddr;
  EFI_I2C_ADDR_MODE       I2CAddrMode;

  EFI_STATUS              Status;
  BOOLEAN *pI2CPrevReadTransfer;
  EFI_HOB_GUID_TYPE       *GuidHob;
//DEBUG((-1, "TpmReadByte called\n"));
  //
  // Get platform type from platform info hob.
  //
  GuidHob      = GetFirstGuidHob (&mGuid2ICPrevReadTransfer);
  if(GuidHob != NULL )
    pI2CPrevReadTransfer = GET_GUID_HOB_DATA (GuidHob);
  else {
#ifdef ADLINK_PEI_COMPILE     
    const EFI_PEI_SERVICES **PeiServices;
    EFI_HOB_I2C_PREV_READ_TRANSFER *pI2cPrevHob;
    PeiServices = GetPeiServicesTablePointer ();

    Status = (*PeiServices)->CreateHob (PeiServices, EFI_HOB_TYPE_GUID_EXTENSION, 
                            sizeof(BOOLEAN) + sizeof (EFI_HOB_GUID_TYPE), (VOID**)&pI2cPrevHob);

    if (EFI_ERROR(Status)) return 0xFF;
    //MemCpy (&(pI2cPrevHob->EfiHobGuidType.Name), &mGuid2ICPrevReadTransfer, sizeof (EFI_GUID));    
    pI2cPrevHob->EfiHobGuidType.Name = mGuid2ICPrevReadTransfer;
    
    pI2cPrevHob->mI2CPrevReadTransfer = FALSE;
    pI2CPrevReadTransfer = &(pI2cPrevHob->mI2CPrevReadTransfer);
#else
#endif    
  }
//DEBUG((-1, "TpmReadByte called: pI2CPrevReadTransfer is %s\n", *pI2CPrevReadTransfer?"TRUE":"FALSE"));
  ReadData  = 0xFF;
  ReadCount = 0;

  //
  // Get platform type from platform info hob.
  //
//<CHW+>   GuidHob       = GetFirstGuidHob (&gEfiPlatformInfoGuid);
//<CHW+>   PlatformInfo  = GET_GUID_HOB_DATA (GuidHob);
//<CHW+>   ASSERT (PlatformInfo != NULL);

  //
  // Only Crosshill platform supports TPM via I2C bus.
  // Otherwise return 0xff.
  //
//<CHW+>   if (PlatformInfo->Type == (EFI_PLATFORM_TYPE) CrossHill) {
    //
    // Locate I2C protocol for TPM I2C access.
    //
//<CHW+>     Status = gBS->LocateProtocol (&gEfiI2CHcProtocolGuid, NULL, (VOID**) &mI2Cbus);
//<CHW+>     if (EFI_ERROR(Status)) {
//<CHW+>       DEBUG ((EFI_D_ERROR, "TpmReadByte(): could not locate I2CHcProtocol\n"));
//<CHW+>     }
//<CHW+>     ASSERT_EFI_ERROR (Status);

    //
    // Setup I2C Slave device address and address mode (7-bit).
    //
    I2CDeviceAddr.I2CDeviceAddress = TPM_I2C_SLAVE_DEVICE_ADDRESS;
    I2CAddrMode = EfiI2CSevenBitAddrMode;

    //
    // Map MMIO address to Infineon I2C address.
    //
    TpmAddress = MapMmioAddr2InfI2CAddr (Address);

    //
    // As recommended by Infineon (SLB9645 I2C Communication protocol application
    // note revision 1.0) retry upto 3 times if TPM status, access or burst count
    // registers return 0xFF.
    //
    while ((ReadData == 0xFF) && (ReadCount < READ_RETRY)) {
      //
      // As recommended by Infineon (SLB9645 I2C Communication protocol application
      // note revision 1.0) wait 250 microseconds between a read and a write transfer.
      //
      //if (*pI2CPrevReadTransfer) {
        MicroSecondDelay (GUARD_TIME);
      //}
//chw+++ Good exmaple for write-read transcation.
      //
      // Write addresss to TPM.
      //
      Data[0] = (UINT8)TpmAddress;
      Status = I2CWriteByte (
                          NULL,
                          I2CDeviceAddr,
                          I2CAddrMode,
                          &Data
                          );

      if (EFI_ERROR(Status)) {
        DEBUG ((EFI_D_INFO, "TpmReadByte(): write to TPM address %0x failed (%r)\n", TpmAddress, Status));
      }

       *pI2CPrevReadTransfer = FALSE;

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
        DEBUG ((EFI_D_INFO, "TpmReadByte(): read from TPM address %0x failed (%r)\n", TpmAddress, Status));
        ReadData = 0xFF;
      } else {
        ReadData = Data[0]; 
      }

      //
      // Only need to retry 3 times for TPM status, access, and burst count registers.
      // If read transfer is to TPM Data FIFO, do not retry, exit loop.
      //
      if (TpmAddress == INFINEON_TPM_DATA_FIFO_0_ADDRESS_DEFAULT) {
        ReadCount = READ_RETRY;
      } else {
        ReadCount++;
      }

       *pI2CPrevReadTransfer = TRUE;
       //DEBUG((-1, "**** TpmReadByte called: pI2CPrevReadTransfer is %s\n", *pI2CPrevReadTransfer?"TRUE":"FALSE"));
    }

    if (EFI_ERROR(Status)) {
      //
      //  Only reads to access register allowed to fail.
      //
      if (TpmAddress != INFINEON_TPM_ACCESS_0_ADDRESS_DEFAULT) {
        DEBUG ((EFI_D_ERROR, "TpmReadByte(): read from TPM address %0x failed\n", TpmAddress));
        ASSERT_EFI_ERROR (Status);
      }
    }

//<CHW+>   } else {
    //
    // Other Quark platforms do not support MMIO access to TPM Device.
    // return 0xff.
    //
//<CHW+>     return (0xFF);
//<CHW+>   }

  return ReadData;
}
