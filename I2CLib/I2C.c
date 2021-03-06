/** @file
  I2C DXE Driver for Quark I2C Controller.
  Follows I2C Controller setup instructions as detailed in
  Quark DataSheet (doc id: 329676) Section 19.1/19.1.3.

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
#include "CommonHeader.h"

#include "I2C.h"

#if 0 
//
// Interface definition of I2C Host Controller Protocol.
//
EFI_I2C_HC_PROTOCOL mI2CbusHc = {
  I2CWriteByte,
  I2CReadByte,
  I2CWriteMultipleByte,
  I2CReadMultipleByte
};
#endif
STATIC EFI_HANDLE mHandle = NULL;
//STATIC UINT16     mSaveCmdReg;
//STATIC UINT32     mSaveBar0Reg;


#define ACPI_IOREG_PM1_TMR      0x08    // Power Management 1 Timer Reg.
#define PM_BASE_ADDRESS 0x400
#define NUM_BITS_IN_ACPI_TIMER  24      // Programmed to 24 not 32
// This is the maximum possible bits in the timer.
// Currently this is 32 according to the spec.
#define MAX_ACPI_TIMER_BITS     32

VOID DumpStructData(UINT8* bData, UINTN nSize){
  UINTN i,j;

  DEBUG((-1, "     |"));

  for(j=0;j<8;j++){
    DEBUG((-1, " %x ", j));
  }
  DEBUG((-1, "-")); 
  for(j=8;j<16;j++){
    DEBUG((-1, " %x ", j));     
  }
  DEBUG((-1, "\n"));  
  for(i=0;i<nSize/16;i++){
    if( i < 10 ) {
      DEBUG((-1, "    %d|", i));
    }
    else if( i < 100) {
      DEBUG((-1, "   %d|", i));
    }
    else if( i < 1000) {
      DEBUG((-1, "  %d|", i));
    }
    else if( i < 10000) {
      DEBUG((-1, " %d|", i));
    }
    else if( i < 100000) {
      DEBUG((-1, "%d|", i));
    }
    else {

    }
    for(j=0;j<8;j++){
      if( bData[i*16+j]<0x10) {
        DEBUG((-1, " %x ", bData[i*16+j]));     
      }
      else {
        DEBUG((-1, "%2x ", bData[i*16+j]));     
      }
    }
    DEBUG((-1, "-"));
    for(j=8;j<16;j++){
      if( bData[i*16+j]<0x10) {
        DEBUG((-1, " %x ", bData[i*16+j]));     
      }
      else {
        DEBUG((-1, "%2x ", bData[i*16+j]));     
      }
    }
    DEBUG((-1, "\n"));  
  }
  if( i*16 < nSize ){
    for(j=i*16;j<nSize;j++){
      if( bData[j]<0x10) {
        DEBUG((-1, " %x ", bData[j]));      
      }
      else {
        DEBUG((-1, "%2x ", bData[j]));      
      }
      if( j%8==7 )
        DEBUG((-1, "-"));
    }
    DEBUG((-1, "\n"));  
  }
}

//
// Handle to install I2C Controller protocol.
//
EFI_STATUS CountTime (
    IN UINTN        DelayTime,
    IN UINT16       BaseAddr ) // Only needs to be 16 bit for I/O address
{
// The following code is to generate delay for specified amount of micro
// seconds using ACPI timer.
    UINTN           TicksNeeded;
    UINT32          TimerValue;
    UINT32          NewTimerValue;
    UINTN           OverFlow;
    UINTN           TheRest;
    UINTN           EndValue;

    // Set up timer to point to the ACPI Timer register
    BaseAddr += ACPI_IOREG_PM1_TMR; // *** PORTING NEEDED

    // There are 3.58 ticks per us, so we have to convert the number of us
    // passed in to the number of ticks that need to pass before the timer has
    // expired convert us to Ticks, don't loose significant figures or as few
    // as possible do integer math in ticks/tens of ns and then divide by 100
    // to get ticks per us
    OverFlow = 0;

//#### TheRest = TicksNeeded = (DelayTime * 358) /100;
    TicksNeeded = DelayTime * 3;            //(DelayTime * 3)
    TicksNeeded += (DelayTime) / 2;         //(DelayTime * 5)/10
    TicksNeeded += (DelayTime * 2) / 25;        //(DelayTime * 8)/100
    TheRest = TicksNeeded;

    // 32 bits corresponds to approz 71 mins  no delay should be that long
    // otherwise get the number of times the counter will have to overflow
    // to delay as long as needed
    if (NUM_BITS_IN_ACPI_TIMER < MAX_ACPI_TIMER_BITS) {
        OverFlow = TicksNeeded / (1 << NUM_BITS_IN_ACPI_TIMER);
        TheRest = TicksNeeded % (1 << NUM_BITS_IN_ACPI_TIMER);
    }

    // Read ACPI Timer
    TimerValue = IoRead32( BaseAddr );

    // Need to adjust the values based off of the start time
    EndValue = TheRest + TimerValue;

    // Check for overflow on addition.  possibly a problem
    if (EndValue < TimerValue) {
        OverFlow++;
    } else {
        if (NUM_BITS_IN_ACPI_TIMER < MAX_ACPI_TIMER_BITS) { 
            // Here make sure that EndValue is less than the max value
            // of the counter
            OverFlow += EndValue / (1 << NUM_BITS_IN_ACPI_TIMER);
            EndValue = EndValue % (1 << NUM_BITS_IN_ACPI_TIMER);
        }
    }

    // Let the timer wrap around as many times as calculated
    while (OverFlow) {
        // read timer amd look to see if the new value read is less than
        // the current timer value.  if this happens the timer overflowed
        NewTimerValue = IoRead32(BaseAddr);

        if (NewTimerValue < TimerValue) OverFlow--;

        TimerValue = NewTimerValue;
    }

    // Now wait for the correct number of ticks that need to occur after
    // all the needed overflows
    while (EndValue > TimerValue) {
        NewTimerValue = IoRead32(BaseAddr);

        // check to see if the timer overflowed.  if it did then
        // the time has elapsed. Because EndValue should be greater than
        // TimerValue
        if (NewTimerValue < TimerValue) break;

        TimerValue = NewTimerValue;
      }

    return EFI_SUCCESS;
}

void
mStall(UINT32 dMicroSeconds)
{
  CountTime(dMicroSeconds, PM_BASE_ADDRESS);
}

/**
  The Called on Protocol Service Entry.

  @return None.

**/
STATIC
VOID
ServiceEntry (
  UINT16     *pSaveCmdReg,
  UINT32     *pSaveBar0Reg
  )
{
  *pSaveBar0Reg = IohMmPci32 (0, I2C_Bus, I2C_Device, I2C_Func, PCI_BAR0);
  if ((*pSaveBar0Reg & B_IOH_I2C_GPIO_MEMBAR_ADDR_MASK) == 0) {

    IohMmPci32(0, I2C_Bus, I2C_Device, I2C_Func, PCI_BAR0) =
            0xD091A000 & B_IOH_I2C_GPIO_MEMBAR_ADDR_MASK;
//          0xFE103000 & B_IOH_I2C_GPIO_MEMBAR_ADDR_MASK;
//chw+      FixedPcdGet32 (PcdIohI2cMmioBase) & B_IOH_I2C_GPIO_MEMBAR_ADDR_MASK;

    //
    // also Save Cmd Register, Setup by InitializeInternal later during xfers.
    //
    *pSaveCmdReg = IohMmPci16 (0, I2C_Bus, I2C_Device, I2C_Func, PCI_CMD);
  }
}

/**
  The Called on Protocol Service Entry.

  @return None.

**/
STATIC
VOID
ServiceExit (
  UINT16     *pSaveCmdReg,
  UINT32     *pSaveBar0Reg
  )
{
  if ((*pSaveBar0Reg & B_IOH_I2C_GPIO_MEMBAR_ADDR_MASK) == 0) {
    IohMmPci16 (0, I2C_Bus, I2C_Device, I2C_Func, PCI_CMD) = *pSaveCmdReg;
    IohMmPci32 (0, I2C_Bus, I2C_Device, I2C_Func, PCI_BAR0) = *pSaveBar0Reg;
  }
}


/**
  The GetI2CIoPortBaseAddress() function gets IO port base address of I2C Controller.

  Always reads PCI configuration space to get MMIO base address of I2C Controller.

  @return The IO port base address of I2C controller.

**/
UINTN
GetI2CIoPortBaseAddress (
  VOID
  )
{
  UINTN     I2CIoPortBaseAddress;

  //
  // Get I2C Memory Mapped registers base address.
  //
  I2CIoPortBaseAddress = IohMmPci32(0, I2C_Bus, I2C_Device, I2C_Func, PCI_BAR0);

  //
  // Make sure that the IO port base address has been properly set.
  //
  ASSERT (I2CIoPortBaseAddress != 0);
  ASSERT (I2CIoPortBaseAddress != 0xFF);

  return I2CIoPortBaseAddress;
}


/**
  The EnableI2CMmioSpace() function enables access to I2C MMIO space.

**/
VOID
EnableI2CMmioSpace (
  VOID
  )
{
  UINT8 PciCmd;

  //
  // Read PCICMD.  Bus=0, Dev=0, Func=0, Reg=0x4
  //
  PciCmd = IohMmPci8(0, I2C_Bus, I2C_Device, I2C_Func, PCI_REG_PCICMD);

  //
  // Enable Bus Master(Bit2), MMIO Space(Bit1) & I/O Space(Bit0)
  //
  PciCmd |= 0x7;
  IohMmPci8(0, I2C_Bus, I2C_Device, I2C_Func, PCI_REG_PCICMD) = PciCmd;

}

/**
  The DisableI2CController() functions disables I2C Controller.

**/
VOID
DisableI2CController (
  VOID
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINT32      Addr;
  UINT32      Data;
  UINT8       PollCount;

  PollCount = 0;

  //
  // Get I2C Memory Mapped registers base address.
  //
  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();

  //
  // Disable the I2C Controller by setting IC_ENABLE.ENABLE to zero
  //
  Addr = (UINT32)(I2CIoPortBaseAddress + I2C_REG_ENABLE);
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_ENABLE;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Read the IC_ENABLE_STATUS.IC_EN Bit to check if Controller is disabled
  //
  Data = 0xFF;
  Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_ENABLE_STATUS;
  Data = *((volatile UINT32 *) (UINTN)(Addr)) & I2C_REG_ENABLE_STATUS;
  while (Data != 0) {
    //
    // Poll the IC_ENABLE_STATUS.IC_EN Bit to check if Controller is disabled, until timeout (TI2C_POLL*MAX_T_POLL_COUNT).
    //
    PollCount++;
    if (PollCount >= MAX_T_POLL_COUNT) {
      break;
    }
    //CHW+ gBS->Stall(TI2C_POLL);
    mStall(TI2C_POLL);
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    Data &= I2C_REG_ENABLE_STATUS;
  }

  //
  // Asset if controller does not enter Disabled state.
  //
  ASSERT (PollCount < MAX_T_POLL_COUNT);

  //
  // Read IC_CLR_INTR register to automatically clear the combined interrupt,
  // all individual interrupts and the IC_TX_ABRT_SOURCE register.
  //
  Addr = (UINT32)(I2CIoPortBaseAddress + I2C_REG_CLR_INT);
  Data = *((volatile UINT32 *) (UINTN)(Addr));

}

/**
  The EnableI2CController() function enables the I2C Controller.

**/
VOID
EnableI2CController (
  VOID
  )
{
  UINTN   I2CIoPortBaseAddress;
  UINT32  Addr;
  UINT32  Data;

  //
  // Get I2C Memory Mapped registers base address.
  //
  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();
  
  //
  // Enable the I2C Controller by setting IC_ENABLE.ENABLE to 1
  //
  Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_ENABLE;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data |= B_I2C_REG_ENABLE;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

/*
  MmioRead32 ( mI2CBaseAddress + R_IC_CLR_TX_ABRT );
  MmioWrite32(mI2CBaseAddress+R_IC_CLK_PARAMS, 1);  
*/
  //
  // Clear overflow and abort error status bits before transactions.
  //
  Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_CLR_RX_OVER;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_CLR_TX_OVER;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_CLR_TX_ABRT;
  Data = *((volatile UINT32 *) (UINTN)(Addr));

  Addr = (UINT32)I2CIoPortBaseAddress + 0x800;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x80020003;    //By Chipset setting.
  /*
DumpStructData((UINT8*)I2CIoPortBaseAddress, 0x100);
DEBUG((-1, "offset 0x800\n"));
DumpStructData((UINT8*)Addr, 0x100);
*/
//Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_RAW_INTR_STAT;
//Data = *((volatile UINT32 *) (UINTN)(Addr));
//DEBUG((-1, " Raw Interrupt Status Register: %x\n", Data));  

}
/*
RETURN_STATUS
I2cBusFrequencySet (
  IN UINTN BusClockHertz
)
{
  DEBUG((EFI_D_INFO,"InputFreq BusClockHertz: %d\r\n",BusClockHertz));
  //
  //  Set the 100 KHz clock divider according to SV result and I2C spec
  //
  MmioWrite32 ( mI2CBaseAddress + R_IC_SS_SCL_HCNT, (UINT16)0x214 );
  MmioWrite32 ( mI2CBaseAddress + R_IC_SS_SCL_LCNT, (UINT16)0x272 );  
  //
  //  Set the 400 KHz clock divider according to SV result and I2C spec
  //
  MmioWrite32 ( mI2CBaseAddress + R_IC_FS_SCL_HCNT, (UINT16)0x50 );
  MmioWrite32 ( mI2CBaseAddress + R_IC_FS_SCL_LCNT, (UINT16)0xAD );
  
  switch ( BusClockHertz ) {
    case 100 * 1000:
      MmioWrite32 ( mI2CBaseAddress + R_IC_SDA_HOLD, (UINT16)0x40);//100K     
      I2cMode = V_SPEED_STANDARD;
      break;
    case 400 * 1000:
      MmioWrite32 ( mI2CBaseAddress + R_IC_SDA_HOLD, (UINT16)0x32);//400K     
      I2cMode = V_SPEED_FAST;
      break;
    default:
      MmioWrite32 ( mI2CBaseAddress + R_IC_SDA_HOLD, (UINT16)0x09);//3.4M
      I2cMode = V_SPEED_HIGH;
  }
  
  //
  //  Select the frequency counter
  //  Enable restart condition,
  //  Enable master FSM, disable slave FSM
  //
  I2cMode |= B_IC_RESTART_EN | B_IC_SLAVE_DISABLE | B_MASTER_MODE;

  return EFI_SUCCESS;
}
*/
/**
  The WaitForStopDet() function waits until I2C STOP Condition occurs,
  indicating transfer completion.

  @retval EFI_SUCCESS           Stop detected.
  @retval EFI_TIMEOUT           Timeout while waiting for stop condition.
  @retval EFI_ABORTED           Tx abort signaled in HW status register.
  @retval EFI_DEVICE_ERROR      Tx or Rx overflow detected.

**/
EFI_STATUS
WaitForStopDet (
  VOID
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINT32      Addr;
  UINT32      Data;
  UINT32       PollCount; //Coding error here.
  EFI_STATUS  Status;

  Status = EFI_SUCCESS;

  PollCount = 0;

  //
  // Get I2C Memory Mapped registers base address.
  //
  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();

  //
  // Wait for STOP Detect.
  //
  Addr = (UINT32)I2CIoPortBaseAddress + I2C_REG_RAW_INTR_STAT;

  do {
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    if ((Data & I2C_REG_RAW_INTR_STAT_TX_ABRT) != 0) {
      Status = EFI_ABORTED;
      break;
    }
    if ((Data & I2C_REG_RAW_INTR_STAT_TX_OVER) != 0) {
      Status = EFI_DEVICE_ERROR;
      break;
    }
    if ((Data & I2C_REG_RAW_INTR_STAT_RX_OVER) != 0) {
      Status = EFI_DEVICE_ERROR;
      break;
    }
    if ((Data & I2C_REG_RAW_INTR_STAT_STOP_DET) != 0) {
      Status = EFI_SUCCESS;
      break;
    }
    //CHW+ gBS->Stall(TI2C_POLL);
    //DEBUG((-1, "Calling mStall %d\n", TI2C_POLL));
    mStall(TI2C_POLL);
    PollCount++;
    if (PollCount >= MAX_STOP_DET_POLL_COUNT) {
      Status = EFI_TIMEOUT;
      break;
    }

  } while (TRUE);

  return Status;
}

/**

  The InitializeInternal() function initialises internal I2C Controller
  register values that are commonly required for I2C Write and Read transfers.

  @param AddrMode     I2C Addressing Mode: 7-bit or 10-bit address.

  @retval EFI_SUCCESS           I2C Operation completed successfully.

**/
EFI_STATUS
InitializeInternal (
  IN  EFI_I2C_ADDR_MODE  AddrMode
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINTN       Addr;
  UINT32      Data;
  EFI_STATUS  Status;

  Status = EFI_SUCCESS;

  //
  // Enable access to I2C Controller MMIO space.
  //
  EnableI2CMmioSpace ();

  //
  // Disable I2C Controller initially
  //
  DisableI2CController ();

  //
  // Get I2C Memory Mapped registers base address.
  //
  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();
/*
  DEBUG((EFI_D_INFO,"InputFreq BusClockHertz: %d\r\n",BusClockHertz));
  //
  //  Set the 100 KHz clock divider according to SV result and I2C spec
  //
  MmioWrite32 ( mI2CBaseAddress + R_IC_SS_SCL_HCNT, (UINT16)0x214 );
  MmioWrite32 ( mI2CBaseAddress + R_IC_SS_SCL_LCNT, (UINT16)0x272 );  
  //
  //  Set the 400 KHz clock divider according to SV result and I2C spec
  //
  MmioWrite32 ( mI2CBaseAddress + R_IC_FS_SCL_HCNT, (UINT16)0x50 );
  MmioWrite32 ( mI2CBaseAddress + R_IC_FS_SCL_LCNT, (UINT16)0xAD );
  
  switch ( BusClockHertz ) {
    case 100 * 1000:
      MmioWrite32 ( mI2CBaseAddress + R_IC_SDA_HOLD, (UINT16)0x40);//100K     
      I2cMode = V_SPEED_STANDARD;
      break;
    case 400 * 1000:
      MmioWrite32 ( mI2CBaseAddress + R_IC_SDA_HOLD, (UINT16)0x32);//400K     
      I2cMode = V_SPEED_FAST;
      break;
    default:
      MmioWrite32 ( mI2CBaseAddress + R_IC_SDA_HOLD, (UINT16)0x09);//3.4M
      I2cMode = V_SPEED_HIGH;
  }
  
  //
  //  Select the frequency counter
  //  Enable restart condition,
  //  Enable master FSM, disable slave FSM
  //
  I2cMode |= B_IC_RESTART_EN | B_IC_SLAVE_DISABLE | B_MASTER_MODE;

*/
  //Set Bus Clock Hertz.
  //
  //  Set the 100 KHz clock divider according to SV result and I2C spec
  //  
  Addr = I2CIoPortBaseAddress + I2C_REG_SS_SCL_HCNT;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x214;
  Addr = I2CIoPortBaseAddress + I2C_REG_SS_SCL_LCNT;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x272;  
  //
  //  Set the 400 KHz clock divider according to SV result and I2C spec
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_FS_SCL_HCNT;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x50;
  Addr = I2CIoPortBaseAddress + I2C_REG_FS_SCL_LCNT;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0xAD;  

  //I2cMode = V_SPEED_STANDARD;
  Addr = I2CIoPortBaseAddress + I2C_REG_SDA_HOLD;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x40;  //100K    

  //
  // Clear START_DET
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_CLR_START_DET;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_CLR_START_DET;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Clear STOP_DET
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_CLR_STOP_DET;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_CLR_STOP_DET;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Set addressing mode to user defined (7 or 10 bit) and
  // speed mode to that defined by PCD (standard mode default).
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_CON;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  // Set Addressing Mode
  if (AddrMode == EfiI2CSevenBitAddrMode) {
    Data &= ~B_I2C_REG_CON_10BITADD_MASTER;
  } else {
    Data |= B_I2C_REG_CON_10BITADD_MASTER;
  }
  // Set Speed Mode
  Data &= ~B_I2C_REG_CON_SPEED;
  if (FALSE) {//CHW+ FeaturePcdGet (PcdI2CFastModeEnabled)) {
    Data |= BIT2;
  } else {
    Data |= BIT1;
  }
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  Data = *((volatile UINT32 *) (UINTN)(Addr));
/*
  MmioWrite32 ( mI2CBaseAddress + R_IC_RX_TL, 0);
  MmioWrite32 ( mI2CBaseAddress + R_IC_TX_TL, 0 );  
*/
/*
  Addr = I2CIoPortBaseAddress + I2C_REG_RX_TL;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x00;  
  Addr = I2CIoPortBaseAddress + I2C_REG_TX_TL;
  *((volatile UINT32 *) (UINTN)(Addr)) = 0x00;  
*/
//Set addressing mode on TAR
//

  Addr = I2CIoPortBaseAddress + I2C_REG_TAR;
  Data = *((volatile UINT32 *) (UINTN)(Addr));

  if (AddrMode == EfiI2CSevenBitAddrMode) {
    Data &= ~BIT12;
  } else {
    Data |= BIT12;
  }

  *((volatile UINT32 *) (UINTN)(Addr)) = Data;
  Data = *((volatile UINT32 *) (UINTN)(Addr));

  return Status;

}

/**

  The WriteByte() function provides a standard way to execute a
  standard single byte write to an IC2 device (without accessing
  sub-addresses), as defined in the I2C Specification.

  @param  I2CAddress      I2C Slave device address
  @param  Value           The 8-bit value to write.

  @retval EFI_SUCCESS           Transfer success.
  @retval EFI_UNSUPPORTED       Unsupported input param.
  @retval EFI_TIMEOUT           Timeout while waiting xfer.
  @retval EFI_ABORTED           Controller aborted xfer.
  @retval EFI_DEVICE_ERROR      Device error detected by controller.

**/
EFI_STATUS
EFIAPI
WriteByte (
  IN  UINTN          I2CAddress,
  IN  UINT8          Value
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINTN       Addr;
  UINT32      Data;
  EFI_STATUS  Status;

  //
  // Get I2C Memory Mapped registers base address
  //
  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();
  //
  // Write to the IC_TAR register the address of the slave device to be addressed
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_TAR;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_TAR;
  Data |= I2CAddress;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Enable the I2C Controller
  //
  EnableI2CController ();

  //
  // Write the data and transfer direction to the IC_DATA_CMD register.
  // Also specify that transfer should be terminated by STOP condition.
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_DATA_CMD;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= 0xFFFFFF00;
  Data |= (UINT8)Value;
  Data &= ~B_I2C_REG_DATA_CMD_RW;
  Data |= B_I2C_REG_DATA_CMD_STOP;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Wait for transfer completion.
  //
  Status = WaitForStopDet ();

  //
  // Ensure I2C Controller disabled.
  //
  DisableI2CController();

  return Status;
}

/**

  The ReadByte() function provides a standard way to execute a
  standard single byte read to an IC2 device (without accessing
  sub-addresses), as defined in the I2C Specification.

  @param  I2CAddress      I2C Slave device address
  @param  ReturnDataPtr   Pointer to location to receive read byte.

  @retval EFI_SUCCESS           Transfer success.
  @retval EFI_UNSUPPORTED       Unsupported input param.
  @retval EFI_TIMEOUT           Timeout while waiting xfer.
  @retval EFI_ABORTED           Controller aborted xfer.
  @retval EFI_DEVICE_ERROR      Device error detected by controller.

**/
EFI_STATUS
EFIAPI
ReadByte (
  IN  UINTN          I2CAddress,
  OUT UINT8          *ReturnDataPtr
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINTN       Addr;
  UINT32      Data;
  EFI_STATUS  Status;

  //
  // Get I2C Memory Mapped registers base address.
  //
  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();

  //
  // Write to the IC_TAR register the address of the slave device to be addressed
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_TAR;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_TAR;
  Data |= I2CAddress;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Enable the I2C Controller
  //
  EnableI2CController ();

  //
  // Write transfer direction to the IC_DATA_CMD register and
  // specify that transfer should be terminated by STOP condition.
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_DATA_CMD;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= 0xFFFFFF00;
  Data |= B_I2C_REG_DATA_CMD_RW;
  Data |= B_I2C_REG_DATA_CMD_STOP;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Wait for transfer completion
  //
  Status = WaitForStopDet ();
  if (!EFI_ERROR(Status)) {

    //
    // Clear RX underflow before reading IC_DATA_CMD.
    //
    Addr = I2CIoPortBaseAddress + I2C_REG_CLR_RX_UNDER;
    Data = *((volatile UINT32 *) (UINTN)(Addr));

    //
    // Obtain and return read data byte from RX buffer (IC_DATA_CMD[7:0]).
    //
    Addr = I2CIoPortBaseAddress + I2C_REG_DATA_CMD;
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    Data &= 0x000000FF;
    *ReturnDataPtr = (UINT8) Data;

    Addr = I2CIoPortBaseAddress + I2C_REG_RAW_INTR_STAT;
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    Data &= I2C_REG_RAW_INTR_STAT_RX_UNDER;
    if (Data != 0) {
      Status = EFI_DEVICE_ERROR;
    }
  }

  //
  // Ensure I2C Controller disabled.
  //
  DisableI2CController ();

  return Status;
}

/**

  The WriteMultipleByte() function provides a standard way to execute
  multiple byte writes to an IC2 device (e.g. when accessing sub-addresses or
  when writing block of data), as defined in the I2C Specification.

  @param I2CAddress   The I2C slave address of the device
                      with which to communicate.

  @param Buffer       Contains the value of byte to be written to the
                      I2C slave device.

  @param Length       No. of bytes to be written.

  @retval EFI_SUCCESS           Transfer success.
  @retval EFI_UNSUPPORTED       Unsupported input param.
  @retval EFI_TIMEOUT           Timeout while waiting xfer.
  @retval EFI_ABORTED           Tx abort signaled in HW status register.
  @retval EFI_DEVICE_ERROR      Tx overflow detected.

**/
EFI_STATUS
EFIAPI
WriteMultipleByte (
  IN  UINTN          I2CAddress,
  IN  UINT8          *WriteBuffer,
  IN  UINTN          Length
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINTN       Index;
  UINTN       Addr;
  UINT32      Data;
  EFI_STATUS  Status;


  if (Length > I2C_FIFO_SIZE) {
    return EFI_UNSUPPORTED;  // Routine does not handle xfers > fifo size.
  }

  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();

  //
  // Write to the IC_TAR register the address of the slave device to be addressed
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_TAR;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_TAR;
  Data |= I2CAddress;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Enable the I2C Controller
  //
  EnableI2CController ();



  //
  // Write the data and transfer direction to the IC_DATA_CMD register.
  // Also specify that transfer should be terminated by STOP condition.
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_DATA_CMD;
  for (Index = 0; Index < Length; Index++) {
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    Data &= 0xFFFFFF00;
    Data |= (UINT8)WriteBuffer[Index];
    Data &= ~B_I2C_REG_DATA_CMD_RW;
/*	
    if (Index == 0)
      Data |= B_I2C_REG_DATA_CMD_RESTART;
    else
      Data &= ~B_I2C_REG_DATA_CMD_RESTART;
  */  
    if (Index == (Length-1)) {
      Data |= B_I2C_REG_DATA_CMD_STOP;
    }

    *((volatile UINT32 *) (UINTN)(Addr)) = Data;
  }

  //
  // Wait for transfer completion
  //

  Status = WaitForStopDet ();

  //
  // Ensure I2C Controller disabled.
  //
  DisableI2CController ();
  return Status;
}

/**

  The ReadMultipleByte() function provides a standard way to execute
  multiple byte writes to an IC2 device (e.g. when accessing sub-addresses or
  when reading block of data), as defined in the I2C Specification (I2C combined
  write/read protocol).

  @param SlaveAddress The I2C slave address of the device
                      with which to communicate.

  @param Buffer       Contains the value of byte data written or read from the
                      I2C slave device.

  @param WriteLength  No. of bytes to be written. In this case data
                      written typically contains sub-address or sub-addresses
                      in Hi-Lo format, that need to be read (I2C combined
                      write/read protocol).

  @param ReadLength   No. of bytes to be read. I

  @param ReadLength   No. of bytes to be read from I2C slave device. 
                      need to be read.

  @param Buffer       Contains the value of byte data read from the
                      I2C slave device.

  @retval EFI_SUCCESS           Transfer success.
  @retval EFI_UNSUPPORTED       Unsupported input param.
  @retval EFI_TIMEOUT           Timeout while waiting xfer.
  @retval EFI_ABORTED           Tx abort signaled in HW status register.
  @retval EFI_DEVICE_ERROR      Rx underflow or Rx/Tx overflow detected.

**/
EFI_STATUS
EFIAPI
ReadMultipleByte (
  IN  UINTN          I2CAddress,
  IN  OUT UINT8      *Buffer,
  IN  UINTN          WriteLength,
  IN  UINTN          ReadLength
  )
{
  UINTN       I2CIoPortBaseAddress;
  UINTN       Index;
  UINTN       Addr;
  UINT32      Data;
  UINT8       PollCount;
  EFI_STATUS  Status;

  if (WriteLength > I2C_FIFO_SIZE || ReadLength > I2C_FIFO_SIZE) {
    return EFI_UNSUPPORTED;  // Routine does not handle xfers > fifo size.
  }

  I2CIoPortBaseAddress = GetI2CIoPortBaseAddress ();

  //
  // Write to the IC_TAR register the address of the slave device to be addressed
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_TAR;
  Data = *((volatile UINT32 *) (UINTN)(Addr));
  Data &= ~B_I2C_REG_TAR;
  Data |= I2CAddress;
  *((volatile UINT32 *) (UINTN)(Addr)) = Data;

  //
  // Enable the I2C Controller
  //
  EnableI2CController ();

  //
  // Write the data (sub-addresses) to the IC_DATA_CMD register.
  //
  Addr = I2CIoPortBaseAddress + I2C_REG_DATA_CMD;
  for (Index = 0; Index < WriteLength; Index++) {
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    Data &= 0xFFFFFF00;
    Data |= (UINT8)Buffer[Index];
    Data &= ~B_I2C_REG_DATA_CMD_RW;
    *((volatile UINT32 *) (UINTN)(Addr)) = Data;
  }

  //
  // Issue Read Transfers for each byte (Restart issued when write/read bit changed).
  //
  for (Index = 0; Index < ReadLength; Index++) {
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    Data |= B_I2C_REG_DATA_CMD_RW;
    // Issue a STOP for last read transfer.
    if (Index == (ReadLength-1)) {
      Data |= B_I2C_REG_DATA_CMD_STOP;
    }
    *((volatile UINT32 *) (UINTN)(Addr)) = Data;
  }

  //
  // Wait for STOP condition.
  //

  Status = WaitForStopDet ();
  if (!EFI_ERROR(Status)) {

    //
    // Poll Receive FIFO Buffer Level register until valid (upto MAX_T_POLL_COUNT times).
    //
    Data = 0;
    PollCount = 0;
    Addr = I2CIoPortBaseAddress + I2C_REG_RXFLR;
    Data = *((volatile UINT32 *) (UINTN)(Addr));
    while ((Data != ReadLength) && (PollCount < MAX_T_POLL_COUNT)) {
      //CHW+ gBS->Stall(TI2C_POLL);
    mStall(TI2C_POLL);
      PollCount++;
      Data = *((volatile UINT32 *) (UINTN)(Addr));
    }

    Addr = I2CIoPortBaseAddress + I2C_REG_RAW_INTR_STAT;
    Data = *((volatile UINT32 *) (UINTN)(Addr));

    //
    // If no timeout or device error then read rx data.
    //
    if (PollCount == MAX_T_POLL_COUNT) {
      Status = EFI_TIMEOUT;
    } else if ((Data & I2C_REG_RAW_INTR_STAT_RX_OVER) != 0) {
      Status = EFI_DEVICE_ERROR;
    } else {

      //
      // Clear RX underflow before reading IC_DATA_CMD.
      //
      Addr = I2CIoPortBaseAddress + I2C_REG_CLR_RX_UNDER;
      Data = *((volatile UINT32 *) (UINTN)(Addr));

      //
      // Read data.
      //
      Addr = I2CIoPortBaseAddress + I2C_REG_DATA_CMD;
      for (Index = 0; Index < ReadLength; Index++) {
        Data = *((volatile UINT32 *) (UINTN)(Addr));
        Data &= 0x000000FF;
        *(Buffer+Index) = (UINT8)Data;
      }
      Addr = I2CIoPortBaseAddress + I2C_REG_RAW_INTR_STAT;
      Data = *((volatile UINT32 *) (UINTN)(Addr));
      Data &= I2C_REG_RAW_INTR_STAT_RX_UNDER;
      if (Data != 0) {
        Status = EFI_DEVICE_ERROR;
      } else {
        Status = EFI_SUCCESS;
      }
    }
  }

  //
  // Ensure I2C Controller disabled.
  //
  DisableI2CController ();

  return Status;
}

/**

  The I2CWriteByte() function is a wrapper function for the WriteByte function.
  Provides a standard way to execute a standard single byte write to an IC2 device
  (without accessing sub-addresses), as defined in the I2C Specification.

  @param This         A pointer to the EFI_I2C_PROTOCOL instance.

  @param SlaveAddress The I2C slave address of the device
                      with which to communicate.

  @param AddrMode     I2C Addressing Mode: 7-bit or 10-bit address.

  @param Length       No. of bytes to be written.

  @param Buffer       Contains the value of byte data to execute to the
                      I2C slave device.


  @retval EFI_SUCCESS           Transfer success.
  @retval EFI_INVALID_PARAMETER  This or Buffer pointers are invalid.
  @retval EFI_UNSUPPORTED       Unsupported input param.
  @retval EFI_TIMEOUT           Timeout while waiting xfer.
  @retval EFI_ABORTED           Controller aborted xfer.
  @retval EFI_DEVICE_ERROR      Device error detected by controller.

**/
EFI_STATUS
EFIAPI
I2CWriteByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN OUT    VOID                    *Buffer
  )
{
  EFI_STATUS Status;
  UINTN      I2CAddress;
  UINT16     mSaveCmdReg; //CHW+
  UINT32     mSaveBar0Reg;  //CHW+

#if 0 //chw+
  if (This != &mI2CbusHc || Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }
#endif
  if (Buffer == NULL)
    return EFI_INVALID_PARAMETER;

  ServiceEntry (&mSaveCmdReg, &mSaveBar0Reg);

  Status = EFI_SUCCESS;

  I2CAddress = SlaveAddress.I2CDeviceAddress;

  Status = InitializeInternal (AddrMode);
  if (!EFI_ERROR(Status)) {
    Status = WriteByte (I2CAddress, *(UINT8 *) Buffer);
  }

  ServiceExit (&mSaveCmdReg, &mSaveBar0Reg);
  return Status;
}

/**

  The I2CReadByte() function is a wrapper function for the ReadByte function.
  Provides a standard way to execute a standard single byte read to an IC2 device
  (without accessing sub-addresses), as defined in the I2C Specification.

  @param This         A pointer to the EFI_I2C_PROTOCOL instance.

  @param SlaveAddress The I2C slave address of the device
                      with which to communicate.

  @param AddrMode     I2C Addressing Mode: 7-bit or 10-bit address.

  @param Buffer       Contains the value of byte data read from the
                      I2C slave device.


  @retval EFI_SUCCESS           Transfer success.
  @retval EFI_INVALID_PARAMETER This or Buffer pointers are invalid.
  @retval EFI_TIMEOUT           Timeout while waiting xfer.
  @retval EFI_ABORTED           Controller aborted xfer.
  @retval EFI_DEVICE_ERROR      Device error detected by controller.


**/
EFI_STATUS
EFIAPI
I2CReadByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN OUT    VOID                    *Buffer
  )
{
  EFI_STATUS Status;
  UINTN      I2CAddress;
  UINT16     mSaveCmdReg; //CHW+
  UINT32     mSaveBar0Reg;  //CHW+
#if 0 //chw+
  if (This != &mI2CbusHc || Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }
#endif
  if (Buffer == NULL)
    return EFI_INVALID_PARAMETER;
  
  ServiceEntry (&mSaveCmdReg, &mSaveBar0Reg);

  Status = EFI_SUCCESS;

  I2CAddress = SlaveAddress.I2CDeviceAddress;

  Status = InitializeInternal (AddrMode);
  if (!EFI_ERROR(Status)) {
    Status = ReadByte (I2CAddress, (UINT8 *) Buffer);
  }
  ServiceExit (&mSaveCmdReg, &mSaveBar0Reg);
  return Status;
}

/**

  The I2CWriteMultipleByte() function is a wrapper function for the WriteMultipleByte() function.
  Provides a standard way to execute multiple byte writes to an IC2 device
  (e.g. when accessing sub-addresses or writing block of data), as defined
  in the I2C Specification.

  @param This         A pointer to the EFI_I2C_PROTOCOL instance.

  @param SlaveAddress The I2C slave address of the device
                      with which to communicate.

  @param AddrMode     I2C Addressing Mode: 7-bit or 10-bit address.

  @param Length       No. of bytes to be written.

  @param Buffer       Contains the value of byte to be written to the
                      I2C slave device.

  @retval EFI_SUCCESS            Transfer success.
  @retval EFI_INVALID_PARAMETER  This, Length or Buffer pointers are invalid.
  @retval EFI_UNSUPPORTED        Unsupported input param.
  @retval EFI_TIMEOUT            Timeout while waiting xfer.
  @retval EFI_ABORTED            Controller aborted xfer.
  @retval EFI_DEVICE_ERROR       Device error detected by controller.

**/
EFI_STATUS
EFIAPI
I2CWriteMultipleByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN UINTN                          *Length,
  IN OUT    VOID                    *Buffer
  )
{
  EFI_STATUS Status;
  UINTN      I2CAddress;
  UINT16     mSaveCmdReg; //CHW+
  UINT32     mSaveBar0Reg;  //CHW+
#if 0 //chw+
  if (This != &mI2CbusHc || Buffer == NULL || Length == NULL) {
    return EFI_INVALID_PARAMETER;
  }
#endif
  if (Buffer == NULL || Length == NULL)
    return EFI_INVALID_PARAMETER;
  
  ServiceEntry (&mSaveCmdReg, &mSaveBar0Reg);
  Status = EFI_SUCCESS;

  I2CAddress = SlaveAddress.I2CDeviceAddress;

  Status = InitializeInternal (AddrMode);
  if (!EFI_ERROR(Status)) {
    Status = WriteMultipleByte (I2CAddress, Buffer, (*Length));
  }

  ServiceExit (&mSaveCmdReg, &mSaveBar0Reg);
  return Status;
}

/**

  The I2CReadMultipleByte() function is a wrapper function for the ReadMultipleByte() function.
  Provides a standard way to execute multiple byte writes to an IC2 device
  (e.g. when accessing sub-addresses or when reading block of data), as defined
  in the I2C Specification (I2C combined write/read protocol).

  @param This         A pointer to the EFI_I2C_PROTOCOL instance.

  @param SlaveAddress The I2C slave address of the device
                      with which to communicate.

  @param AddrMode     I2C Addressing Mode: 7-bit or 10-bit address.

  @param WriteLength  No. of bytes to be written. In this case data
                      written typically contains sub-address or sub-addresses
                      in Hi-Lo format, that need to be read (I2C combined
                      write/read protocol).

  @param ReadLength   No. of bytes to be read from I2C slave device.
                      need to be read.

  @param Buffer       Contains the value of byte data read from the
                      I2C slave device.

  @retval EFI_SUCCESS            Transfer success.
  @retval EFI_INVALID_PARAMETER  This, WriteLength, ReadLength or Buffer
                                 pointers are invalid.
  @retval EFI_UNSUPPORTED        Unsupported input param.
  @retval EFI_TIMEOUT            Timeout while waiting xfer.
  @retval EFI_ABORTED            Controller aborted xfer.
  @retval EFI_DEVICE_ERROR       Device error detected by controller.

**/
EFI_STATUS
EFIAPI
I2CReadMultipleByte (
  IN CONST  EFI_I2C_HC_PROTOCOL     *This,
  IN        EFI_I2C_DEVICE_ADDRESS  SlaveAddress,
  IN        EFI_I2C_ADDR_MODE       AddrMode,
  IN UINTN                          *WriteLength,
  IN UINTN                          *ReadLength,
  IN OUT    VOID                    *Buffer
  )
{
  EFI_STATUS Status;
  UINTN      I2CAddress;
  UINT16     mSaveCmdReg; //CHW+
  UINT32     mSaveBar0Reg;  //CHW+  
#if 0 //CHW+
  if (This != &mI2CbusHc) {
    return EFI_INVALID_PARAMETER;
  }
#endif   
  if (Buffer == NULL || WriteLength == NULL || ReadLength == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  ServiceEntry (&mSaveCmdReg, &mSaveBar0Reg);

  Status = EFI_SUCCESS;

  I2CAddress = SlaveAddress.I2CDeviceAddress;
  Status = InitializeInternal (AddrMode);
  if (!EFI_ERROR(Status)) {
    Status = ReadMultipleByte (I2CAddress, Buffer, (*WriteLength), (*ReadLength));
  }
  ServiceExit (&mSaveCmdReg, &mSaveBar0Reg);
  return Status;
}

#if 0 
/**
  Entry point to the DXE Driver that produces the I2C Controller Protocol.

  @param  ImageHandle      ImageHandle of the loaded driver.
  @param  SystemTable      Pointer to the EFI System Table.

  @retval EFI_SUCCESS      The entry point of I2C DXE driver is executed successfully.
  @retval !EFI_SUCESS      Some error occurs in the entry point of I2C DXE driver.

**/
EFI_STATUS
EFIAPI
InitializeI2C (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS    Status;
  Status = EFI_SUCCESS;

  Status = gBS->InstallMultipleProtocolInterfaces (
                  &mHandle,
                  &gEfiI2CHcProtocolGuid,
                  &mI2CbusHc,
                  NULL
                  );

  ASSERT_EFI_ERROR (Status);

  return Status;
}
#endif
