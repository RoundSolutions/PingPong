/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
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

#include <xc.h>
#include <sys/attribs.h>
#include "app.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void __ISR(_CHANGE_NOTICE_C_VECTOR, ipl4AUTO) _IntHandlerChangeNotification_PortC(void)
{
    PulseCounter1++;
    unsigned int __attribute__ ((unused)) temp;

    /* Read port to clear mismatch on change notice pins */
    temp = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_C);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_C);
}
void __ISR(_CHANGE_NOTICE_E_VECTOR, ipl4AUTO) _IntHandlerChangeNotification_PortE(void)
{
    PulseCounter2++;
    unsigned int __attribute__ ((unused)) temp;

    /* Read port to clear mismatch on change notice pins */
    temp = PLIB_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_CHANGE_NOTICE_E);
}

void __ISR(_TIMER_1_VECTOR, ipl4AUTO) _IntHandlerDrvTmrInstance0(void)
{

    DRV_TMR_Tasks_ISR(sysObj.drvTmr0);

}
void __ISR(_TIMER_5_VECTOR, ipl4AUTO) _IntHandlerDrvTmrInstance1(void)
{

    DRV_TMR_Tasks_ISR(sysObj.drvTmr1);

}
  
void __ISR(_UART2_TX_VECTOR, ipl4AUTO) _IntHandlerDrvUsartTransmitInstance0(void)
{

    DRV_USART_TasksTransmit(sysObj.drvUsart0);
}
void __ISR(_UART2_RX_VECTOR, ipl4AUTO) _IntHandlerDrvUsartReceiveInstance0(void)
{

    DRV_USART_TasksReceive(sysObj.drvUsart0);

}
void __ISR(_UART2_FAULT_VECTOR, ipl1AUTO) _IntHandlerDrvUsartErrorInstance0(void)
{

    DRV_USART_TasksError(sysObj.drvUsart0);

}
 
 

 
void __ISR(_UART1_TX_VECTOR, ipl4AUTO) _IntHandlerDrvUsartTransmitInstance1(void)
{

    DRV_USART_TasksTransmit(sysObj.drvUsart1);

}
void __ISR(_UART1_RX_VECTOR, ipl4AUTO) _IntHandlerDrvUsartReceiveInstance1(void)
{

    DRV_USART_TasksReceive(sysObj.drvUsart1);

}
void __ISR(_UART1_FAULT_VECTOR, ipl1AUTO) _IntHandlerDrvUsartErrorInstance1(void)
{

    DRV_USART_TasksError(sysObj.drvUsart1);

}
 
 
 

 

 
 
  
void __ISR(_SPI4_RX_VECTOR, ipl4AUTO) _IntHandlerSPIRxInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void __ISR(_SPI4_TX_VECTOR, ipl4AUTO) _IntHandlerSPITxInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
void __ISR(_SPI4_FAULT_VECTOR, ipl1AUTO) _IntHandlerSPIFaultInstance0(void)
{
    DRV_SPI_Tasks(sysObj.spiObjectIdx0);
}
 
void __ISR(_I2C2_MASTER_VECTOR, ipl4AUTO) _IntHandlerDrvI2CMasterInstance0(void)
{
    DRV_I2C_Tasks(sysObj.drvI2C0);
}

void __ISR(_I2C2_BUS_VECTOR, ipl1AUTO) _IntHandlerDrvI2CErrorInstance0(void)
{
    SYS_ASSERT(false, "I2C Driver Instance 0 Error");
}
     
   

  
 
  
 



 

  
  
  
  
  
 

  
  
  
 void __ISR(_RTCC_VECTOR, ipl2AUTO) _IntHandlerRTCC(void)
{
 	
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_RTCC);
	
	
}

	
	
	
void __ISR(_USB_VECTOR, ipl4AUTO) _IntHandlerUSBInstance0(void)
{

    DRV_USBHS_Tasks_ISR(sysObj.drvUSBObject);
            
}
void __ISR ( _USB_DMA_VECTOR,ipl4AUTO) _IntHandlerUSBInstance0_USBDMA ( void )
{

    DRV_USBHS_Tasks_ISR_USBDMA(sysObj.drvUSBObject);

}


 
/*******************************************************************************
 End of File
*/

