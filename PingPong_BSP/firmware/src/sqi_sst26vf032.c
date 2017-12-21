/*******************************************************************************
  SQI functions

  File Name:
    sqi_sst26vf032.c

  Summary:
    SQI functions

  Description:
    This is the sample driver that implements all the SQI functions.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013 released Microchip Technology Inc.  All rights reserved.

//Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "sys/kmem.h"
#include "app.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Variables
// *****************************************************************************
// *****************************************************************************
uint32_t __attribute__((coherent)) JedecID;
uint32_t __attribute__((coherent)) JedecIDGolden;

// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Read
// *****************************************************************************
// *****************************************************************************
static uint32_t APP_ReadCoreTimer()
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return(timer);
}

// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Sart
// *****************************************************************************
// *****************************************************************************
static void APP_StartCoreTimer(uint32_t period)
{
    /* Reset the coutner */
    volatile uint32_t loadZero = 0;

    asm volatile("mtc0   %0, $9" : "+r"(loadZero));
    asm volatile("mtc0   %0, $11" : "+r" (period));
}
// *****************************************************************************
// *****************************************************************************
// Function: Core Timer Delay
// *****************************************************************************
// *****************************************************************************
static void APP_CoreTimer_Delay(uint32_t delayValue)
{
    while ((APP_ReadCoreTimer() <= delayValue))
        asm("nop");
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI PIO Read
// *****************************************************************************
// *****************************************************************************
int SQI_DMA_Read (uint32_t address)
{
    sqiDMADesc pSqiDMADesc1, pSqiDMADesc2, pSqiDMADesc3;
    uint32_t checkLoop;
    uint8_t *readBufInByte = (uint8_t *) BD_BUFFER2_ADDR;
    uint8_t * writeBuf = (uint8_t *) WRITE_BUF_ADDR;
    unsigned int *buf1VAAddr = (unsigned int *) BD_BUFFER1_ADDR;
    unsigned int *buf1PAAddr = (unsigned int *) BD_BUFFER1_PA_ADDR;
    unsigned int *buf2PAAddr = (unsigned int *) BD_BUFFER2_PA_ADDR;
    uint8_t tempAddress1, tempAddress2, tempAddress3;
    uint32_t errCount = 1;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    // Initialize data in the Tx buffer
    *buf1VAAddr++ = address << 8 | SST26VF032_FASTREAD;
    *buf1VAAddr = 0x00000000;

    // Setup Buffer Descriptors in Memory
    //BD1
    PIC32_UNCACHED_VAR(pSqiDMADesc1.BDCon) = 0x90800004;
    PIC32_UNCACHED_VAR(pSqiDMADesc1.BDStat) = 0;
    PIC32_UNCACHED_VAR(pSqiDMADesc1.BDAddr) = (unsigned int*) buf1PAAddr;
    PIC32_UNCACHED_VAR(pSqiDMADesc1.nextBDAddr) = (struct sqiDMADesc *)KVA_TO_PA(&pSqiDMADesc2);

    //BD2
    PIC32_UNCACHED_VAR(pSqiDMADesc2.BDCon) = 0x90900001;
    PIC32_UNCACHED_VAR(pSqiDMADesc2.BDStat) = 0;
    PIC32_UNCACHED_VAR(pSqiDMADesc2.BDAddr) = (unsigned int*) buf1PAAddr+4;
    PIC32_UNCACHED_VAR(pSqiDMADesc2.nextBDAddr) = (struct sqiDMADesc *)KVA_TO_PA(&pSqiDMADesc3);

    //BD2
    PIC32_UNCACHED_VAR(pSqiDMADesc3.BDCon) = 0x90950100;
    PIC32_UNCACHED_VAR(pSqiDMADesc3.BDStat) = 0;
    PIC32_UNCACHED_VAR(pSqiDMADesc3.BDAddr) = (unsigned int*) buf2PAAddr;
    PIC32_UNCACHED_VAR(pSqiDMADesc3.nextBDAddr) = 0;

    // Enable BDDONE flag
    PLIB_SQI_InterruptEnable(SQI_ID_0,SQI_BDDONE);

    // Setup SQI transfer thresholds
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0,4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0,0x18);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0,4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0,0x18);

    //BD_BUFFER1_ADDR address pointer (Base buffer descriptor)
    PLIB_SQI_DMABDBaseAddressSet(SQI_ID_0, (void *)(KVA_TO_PA(&pSqiDMADesc1)));

    // Setup poll control value
    PLIB_SQI_DMABDPollCounterSet(SQI_ID_0,0);

    //Configure DMA mode
    PLIB_SQI_TransferModeSet(SQI_ID_0, SQI_XFER_MODE_DMA);

    // Enable and start DMA
    PLIB_SQI_DMAEnable(SQI_ID_0);
    PLIB_SQI_DMABDFetchStart(SQI_ID_0);

    PLIB_SQI_DMADisable(SQI_ID_0);

    //Check to see if BD is finished
    while(!PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_BDDONE));

    for (checkLoop=0; checkLoop <256; checkLoop++){
        if (*readBufInByte++ != *writeBuf++)
            errCount++;
    }

    return errCount;
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI PIO Write
// *****************************************************************************
// *****************************************************************************
void SQI_PIO_PageWrite (uint32_t address)
{
    uint32_t writeLoop, bufLoop;
    uint8_t writeLoopChar = 0;
    uint8_t * writeBufAddrChar = (uint8_t *) WRITE_BUF_ADDR;
    uint8_t * txBufChar  = (uint8_t *) SQI_TXBUF_ADDR;
    uint8_t tempAddress1, tempAddress2, tempAddress3;

    // Setup transmit data
    for (writeLoop=0;writeLoop < 256; writeLoop++)
        *writeBufAddrChar++= writeLoopChar++;

    writeBufAddrChar = (uint8_t *) WRITE_BUF_ADDR;

    // Address manipulation (LaZ logic)
    tempAddress1 = (uint8_t) (address >> 16);
    tempAddress2 = (uint8_t) (address >> 8);
    tempAddress3 = (uint8_t) address;
    address = tempAddress1 | tempAddress2 << 8 | tempAddress3 << 16;

    //Setup SQI transmit buffer thresholds
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0,4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0,4);

    // SQI Transfer Configuration
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send WEN command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);

    //Write the command to the transfer buffer {WEN,NOP,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032_WEN << 24) | 0x00000000);

    //Start Write
    // Setup control word to send PAGE WRITE command
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send ADDRESS
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,3);
    // Setup control word to send 256 bytes of DATA
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,0x100);

    // Write the command to the transfer buffer
    PLIB_SQI_TransmitData(SQI_ID_0, address << 8 | SST26VF032_PAGEWRITE);

    //Write the data to flash
    for (writeLoop=0; writeLoop < 16; writeLoop++){
        while (!(PLIB_SQI_InterruptFlagGet(SQI_ID_0,SQI_TXTHR)));
        for (bufLoop=0; bufLoop < MAX_WRITE_BUF_DEPTH; bufLoop++){
            *txBufChar = *writeBufAddrChar++;          // Next byte of write data
        }
    }

    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(180000);
}

// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash ID Read
// *****************************************************************************
// *****************************************************************************
int SQI_FlashID_Read (void)
{
    JedecIDGolden = SST26VF032_JEDECID;

    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send NOP command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send EQIO command
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to send FAST READ command
    PLIB_SQI_ControlWordSet(SQI_ID_0,0,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_TRANSMIT,1);
    // Setup control word to read the Jedec ID
    PLIB_SQI_ControlWordSet(SQI_ID_0,1,SQI_CS_1,SQI_LANE_QUAD,SQI_CMD_RECEIVE,4);

    //Write the command to the transfer buffer {WEN,EQIO,NOP,NOP}
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032_QJID << 24) | 0x00000000);

    while (PLIB_SQI_InterruptFlagGet(SQI_ID_0, SQI_RXTHR) == false);

    JedecID = PLIB_SQI_ReceiveData(SQI_ID_0);

    if (JedecID != JedecIDGolden)
        return false;
    else
        return true;

}

// *****************************************************************************
// *****************************************************************************
// Section: SQI Flash Setup
// *****************************************************************************
// *****************************************************************************
void SQI_Flash_Setup (void)
{
    /* Setup CON, TX and RX buffer thresholds */
    PLIB_SQI_ControlBufferThresholdSet(SQI_ID_0, 1);
    PLIB_SQI_TxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdSet(SQI_ID_0, 4);
    PLIB_SQI_TxBufferThresholdIntSet(SQI_ID_0, 4);
    PLIB_SQI_RxBufferThresholdIntSet(SQI_ID_0, 4);

    /* Setup control buffer to set SQI Flash in quad lane mode and  send erase the flash command  */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);   /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_SINGLE, SQI_CMD_TRANSMIT, 1);   /* EQIO */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WREN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* ERASE */

    /* Send necessary commands to erase the flash */
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032_ERASE << 24) |
                         (SST26VF032_WEN << 16) |
                         (SST26VF032_EQIO << 8) |
                         0x00);

    /* Wait for 50ms for the erase to complete */
    APP_StartCoreTimer(0);
    APP_CoreTimer_Delay(5000000);

    /* Setup control buffer to send block unprotect command */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* NOP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 1, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* WREN */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 1);     /* BLKUP */
    PLIB_SQI_ControlWordSet(SQI_ID_0, 0, SQI_CS_1, SQI_LANE_QUAD, SQI_CMD_TRANSMIT, 12);     /* block unprotect bits */

    /* Send necessary commands to unprotect the flash blocks*/
    PLIB_SQI_TransmitData(SQI_ID_0, (SST26VF032_BLKUP << 24) | (SST26VF032_WEN << 16) | 0x00000000);
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
    PLIB_SQI_TransmitData(SQI_ID_0, 0x00000000);
}

/*******************************************************************************
 End of File
*/
