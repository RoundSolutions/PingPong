/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "app.h"
#include "../framework/peripheral/peripheral.h"
#include "Math.h"
#include "peripheral/rtcc/plib_rtcc.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions

unsigned int p_USBDebugMessageSize;
int p_DebugMessagesCount;
uint8_t APP_MAKE_BUFFER_DMA_READY p_USBDebugMessage[2048];

uint8_t APP_MAKE_BUFFER_DMA_READY readBuffer[APP_READ_BUFFER_SIZE];

// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA_USB appData_USB;
APP_DATA_GPS appData_GPS;
APP_DATA_XE910 appData_XE910;
APP_DATA_ACCEL appData_ACCEL;
APPData_Flash appData_Flash;
APPData_RTCC appData_RTCC;
APPDATA_MCP3910 appData_MCP3910;
DRV_I2C_BUFFER_EVENT operationStatus;
APP_DATA_IC appDataIC;
ACCELDATA accelData;
MAGDATA magData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
void APP_I2C_MasterOpEndCallback (DRV_I2C_BUFFER_EVENT event,
                                DRV_I2C_BUFFER_HANDLE bufferHandle,
                                uintptr_t context)
{
    switch(event)
    {
        case DRV_I2C_SEND_STOP_EVENT:
            DRV_I2C_StopEventSend(appData_ACCEL.drvI2CHandle);
            break;
        case DRV_I2C_SEND_RESTART_EVENT:
            DRV_I2C_RestartEventSend(appData_ACCEL.drvI2CHandle);
            break;
        case DRV_I2C_BUFFER_EVENT_COMPLETE:
              GREEN_LED_Toggle();
            switch(appData_ACCEL.state)
            {
                case ACCEL_RD_WHOAMI:if(appData_ACCEL.drvI2CTXbuffer[0]==0xC7){/*checking device ID*/}
                    appData_ACCEL.state=ACCEL_STDBY; break;
                case ACCEL_STDBY:appData_ACCEL.state=ACCEL_CONFIGMAG_REG1;break;
                case ACCEL_CONFIGMAG_REG1:appData_ACCEL.state=ACCEL_CONFIGMAG_REG2;break;
                case ACCEL_CONFIGMAG_REG2:appData_ACCEL.state=ACCEL_XYZ_CONFIG;break;
                case ACCEL_XYZ_CONFIG:appData_ACCEL.state=ACCEL_CONFIG_REG1;break;
                case ACCEL_CONFIG_REG1:appData_ACCEL.state=ACCEL_ReadData;break;
                case ACCEL_ReadData:AccelDataReady = true;break;
            }

            break;
        default:
            break;
    }

}
/************************* SPI EVENT HANDLER ***************************/

void APP_BufferEventHandlerSPI1(DRV_SPI_BUFFER_EVENT buffEvent,
                            DRV_SPI_BUFFER_HANDLE hBufferEvent,
                            void* context )
{
    switch(buffEvent)
    {   
        /* Buffer event is completed successfully. Data transmitted / received*/
        case DRV_SPI_BUFFER_EVENT_COMPLETE:
        {   
            
            if((context == appData_MCP3910.gBufferContext.SPI1TxBufferRef) &&
                                  (appData_MCP3910.state == APP_STATE_WAIT_CONFIG1))
            {
                appData_MCP3910.state = APP_STATE_READ_CH0;
                
                PLIB_PORTS_PinSet(ADC_CS);
                       
            }
            else if((context == appData_MCP3910.gBufferContext.SPI1RxBufferRef) &&
                            (appData_MCP3910.state == APP_STATE_WAIT_READ_COMPLETE))
            {
                appData_MCP3910.SPI_Buffer_Index += 3;
                
                appData_MCP3910.p_SPITimeout = L_GetTickCounter();
                
                appData_MCP3910.state = APP_STATE_READ_DONE;
                PLIB_PORTS_PinSet(ADC_CS);
                
            }
            else if((context == appData_MCP3910.gBufferContext.SPI1TxBufferRef) &&
                                        (appData_MCP3910.state == APP_STATE_RESUME))
            {
                
                appData_MCP3910.p_SPITimeout = L_GetTickCounter();
                
                PLIB_PORTS_PinSet(ADC_CS);
            }
                                          
        }
        break;

        /* Buffer event has some error */
        case DRV_SPI_BUFFER_EVENT_ERROR:
        {    /* Write-Read failed*/
         
        }
        break;

        default:
            break;
    }
}
void APP_BufferEventHandlerXE910Usart(DRV_USART_BUFFER_EVENT buffEvent,DRV_USART_BUFFER_HANDLE hBufferEvent,uintptr_t context )
{
switch(buffEvent)
    {
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            switch(appData_XE910.XE910ResponseState)
            {
                case AT_RESPONSE:   break;
                case AT_GPIO_RESPONSE:   break;
                case AT_CMEE_RESPONSE:   break;
                case AT_CPIN_RESPONSE:   break;
                case AT_CGSN_RESPONSE:   break;
                case AT_ANDC1_RESPONSE:   break;
                case AT_SELINT_RESPONSE:   break;
                case AT_SCFG_RESPONSE:   break;
                case AT_CGDCONT_RESPONSE:   break;
                case AT_SGACT_Q_RESPONSE:   break;
                case AT_SGACT_RESPONSE:   break;
                case AT_MONI_RESPONSE:   break;
                case AT_SD_RESPONSE:   break;
                default: break;
            }
        }
        break;
        case DRV_USART_BUFFER_EVENT_ERROR:
            if(U1STAbits.OERR==1)
                 U1STAbits.OERR=0;
            break;

       default:
            break;
    }
}
void APP_BufferEventHandlerUsart2(DRV_USART_BUFFER_EVENT buffEvent,DRV_USART_BUFFER_HANDLE hBufferEvent,uintptr_t context )
{
    size_t processedBytes;
    switch(buffEvent)
    {
        /* Buffer event is completed successfully */
        case DRV_USART_BUFFER_EVENT_COMPLETE:
        {
            processedBytes = DRV_USART_BufferProcessedSizeGet(hBufferEvent);
            appData_GPS.drvUsart2RxBuffer_Index = processedBytes;
            GetRMC_GGMAMessags();
        }
        break;

        /* Buffer event has some error */
        case DRV_USART_BUFFER_EVENT_ERROR:

            break;

       default:
            break;
    }
}
/*******************************************************
 * USB CDC Device Events - Application Event Handler
 *******************************************************/

USB_DEVICE_CDC_EVENT_RESPONSE APP_USBDeviceCDCEventHandler
(
    USB_DEVICE_CDC_INDEX index ,
    USB_DEVICE_CDC_EVENT event ,
    void * pData,
    uintptr_t userData
)
{
    APP_DATA_USB * appDataObject;
    appDataObject = (APP_DATA_USB *)userData;
    USB_CDC_CONTROL_LINE_STATE * controlLineStateData;
    USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *ReceivedData;

    switch ( event )
    {
        case USB_DEVICE_CDC_EVENT_GET_LINE_CODING:

            /* This means the host wants to know the current line
             * coding. This is a control transfer request. Use the
             * USB_DEVICE_ControlSend() function to send the data to
             * host.  */

            USB_DEVICE_ControlSend(appDataObject->deviceHandle,
                    &appDataObject->getLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_LINE_CODING:

            /* This means the host wants to set the line coding.
             * This is a control transfer request. Use the
             * USB_DEVICE_ControlReceive() function to receive the
             * data from the host */

            USB_DEVICE_ControlReceive(appDataObject->deviceHandle,
                    &appDataObject->setLineCodingData, sizeof(USB_CDC_LINE_CODING));

            break;

        case USB_DEVICE_CDC_EVENT_SET_CONTROL_LINE_STATE:

            /* This means the host is setting the control line state.
             * Read the control line state. We will accept this request
             * for now. */

            controlLineStateData = (USB_CDC_CONTROL_LINE_STATE *)pData;
            appDataObject->controlLineStateData.dtr = controlLineStateData->dtr;
            appDataObject->controlLineStateData.carrier = controlLineStateData->carrier;

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);

            break;

        case USB_DEVICE_CDC_EVENT_SEND_BREAK:

            /* This means that the host is requesting that a break of the
             * specified duration be sent. Read the break duration */

            appDataObject->breakData = ((USB_DEVICE_CDC_EVENT_DATA_SEND_BREAK *)pData)->breakDuration;
            break;

        case USB_DEVICE_CDC_EVENT_READ_COMPLETE:

            /* This means that the host has sent some data*/
            ReceivedData = (USB_DEVICE_CDC_EVENT_DATA_READ_COMPLETE *) pData;
            RXDatalength = ReceivedData->length;
            appDataObject->isReadComplete = true;
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_RECEIVED:

            /* The data stage of the last control transfer is
             * complete. For now we accept all the data */

            USB_DEVICE_ControlStatus(appDataObject->deviceHandle, USB_DEVICE_CONTROL_STATUS_OK);
            break;

        case USB_DEVICE_CDC_EVENT_CONTROL_TRANSFER_DATA_SENT:

            /* This means the GET LINE CODING function data is valid. We dont
             * do much with this data in this demo. */
            break;

        case USB_DEVICE_CDC_EVENT_WRITE_COMPLETE:

            /* This means that the data write got completed. We can schedule
             * the next read. */

            appDataObject->isWriteComplete = true;
            break;

        default:
            break;
    }

    return USB_DEVICE_CDC_EVENT_RESPONSE_NONE;
}


/***********************************************
 * Application USB Device Layer Event Handler.
 ***********************************************/
void APP_USBDeviceEventHandler ( USB_DEVICE_EVENT event, void * eventData, uintptr_t context )
{
    USB_DEVICE_EVENT_DATA_CONFIGURED *configuredEventData;

    switch ( event )
    {
        case USB_DEVICE_EVENT_SOF:

             appData_USB.sofEventHasOccurred = true;
            break;

        case USB_DEVICE_EVENT_RESET:

            /* Update LED to show reset state */
//            BSP_LEDOn ( APP_USB_LED_1 );
//            BSP_LEDOn ( APP_USB_LED_2 );
//            BSP_LEDOff ( APP_USB_LED_3 );

            appData_USB.isConfigured = false;

            break;

        case USB_DEVICE_EVENT_CONFIGURED:

            /* Check the configuratio. We only support configuration 1 */
            configuredEventData = (USB_DEVICE_EVENT_DATA_CONFIGURED*)eventData;
            if ( configuredEventData->configurationValue == 1)
            {
                /* Update LED to show configured state */
//                BSP_LEDOff ( APP_USB_LED_1 );
//                BSP_LEDOff ( APP_USB_LED_2 );
//                BSP_LEDOn ( APP_USB_LED_3 );

                /* Register the CDC Device application event handler here.
                 * Note how the appData object pointer is passed as the
                 * user data */

                USB_DEVICE_CDC_EventHandlerSet(USB_DEVICE_CDC_INDEX_0, APP_USBDeviceCDCEventHandler, (uintptr_t)&appData_USB);

                /* Mark that the device is now configured */
                appData_USB.isConfigured = true;

            }
            break;

        case USB_DEVICE_EVENT_POWER_DETECTED:

            /* VBUS was detected. We can attach the device */
            USB_DEVICE_Attach(appData_USB.deviceHandle);
            break;

        case USB_DEVICE_EVENT_POWER_REMOVED:

            /* VBUS is not available any more. Detach the device. */
            USB_DEVICE_Detach(appData_USB.deviceHandle);
            break;

        case USB_DEVICE_EVENT_SUSPENDED:

            /* Switch LED to show suspended state */
//            BSP_LEDOff ( APP_USB_LED_1 );
//            BSP_LEDOn ( APP_USB_LED_2 );
//            BSP_LEDOn ( APP_USB_LED_3 );
            break;

        case USB_DEVICE_EVENT_RESUMED:
        case USB_DEVICE_EVENT_ERROR:
        default:
            break;
    }
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/*****************************************************
 * This function is called in every step of the
 * application state machine.
 *****************************************************/

bool APP_StateReset(void)
{
    /* This function returns true if the device
     * was reset  */

    bool retVal;

    if(appData_USB.isConfigured == false)
    {
        appData_USB.state = APP_STATE_WAIT_FOR_CONFIGURATION;
        appData_USB.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData_USB.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
        appData_USB.isReadComplete = true;
        appData_USB.isWriteComplete = true;
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return(retVal);
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{


    /* Place the App state machine in its initial state. */
    appData_USB.state = APP_STATE_INIT;
    appData_USB.deviceHandle = USB_DEVICE_HANDLE_INVALID ;

     /* Device configured status */
     appData_USB.isConfigured = false;

     /* Initial get line coding state */
     appData_USB.getLineCodingData.dwDTERate = 9600;
     appData_USB.getLineCodingData.bParityType =  0;
     appData_USB.getLineCodingData.bParityType = 0;
     appData_USB.getLineCodingData.bDataBits = 8;

     /* Read Transfer Handle */
     appData_USB.readTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

     /* Write Transfer Handle */
     appData_USB.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

     /* Intialize the read complete flag */
     appData_USB.isReadComplete = true;

     /*Initialize the write complete flag*/
     appData_USB.isWriteComplete = true;

     /* Reset other flags */
     appData_USB.sofEventHasOccurred = false;

     /* Set up the read buffer */
     appData_USB.readBuffer = &readBuffer[0];
     
     appData_GPS.state = APP_STATE_INIT;
     appData_GPS.drvUsart2Handle= DRV_HANDLE_INVALID;
     appData_GPS.drvUsart2RxBuffer_Index = 0;
     appData_GPS.FoundRMC =false;
     appData_GPS.FoundGGA = false;
     appData_GPS.GPRMC_Index = 0;
     appData_GPS.GPGGA_Index = 0;
    /* Initialize the timer*/
     p_TickCounter = 0;
     p_GeneralCounter = 0;
     p_GPSTimeout = 0;
    Timer_handle = DRV_TMR_Open(DRV_TMR_INDEX_1, DRV_IO_INTENT_EXCLUSIVE);
    DRV_TMR_AlarmPeriod32BitSet(Timer_handle,39062);
    DRV_TMR_Alarm32BitRegister (Timer_handle, 39062, true, 0, Timer_CounterCallback );
    DRV_TMR_Start(Timer_handle);
    
    appData_XE910.XE910_State = XE910_POWER_ON;
    appData_XE910.XE910_RXBufferIndex = 0;
    appData_XE910.XE910_Buffer_Index = 0;
    appData_XE910.drvXE910UsartHandle = DRV_HANDLE_INVALID;
    appData_XE910.XE910_Buffer_Index = 0;
    appData_XE910.XE910_CumulocityBufferIndex = 0;
    appData_ACCEL.state = APP_STATE_INIT;
    AccelDataReady = false;
    initADC();
    PLIB_SQI_ConfigWordSet(SQI_ID_0,
        		   1,
                	   SQI_CS_OEN_1,
			   SQI_DATA_OEN_QUAD,
			   0, // Resets control, transmit, receive buffers and state machines
			   1, // Burst Enable (always set to '1')
			   0, // SQID2 doesn’t act as HOLD# signal in single and dual lane modes
                           0, // SQID3 doesn’t act as WP# signal in single and dual lane modes
			   0, // Receive latch is not active in transmit mode
                           SQI_DATA_FORMAT_MSBF,
			   SQI_DATA_MODE_3,
			   SQI_XFER_MODE_PIO
			  );
    // Configure SQI1CLKCON
    PLIB_SQI_ClockDividerSet(SQI_ID_0, CLK_DIV_2);
    PLIB_SQI_ClockEnable(SQI_ID_0);
    appData_Flash.state = APP_STATE_INIT;
    appDataIC.state = APP_STATE_INIT;
    appData_MCP3910.state = APP_STATE_INIT;
    appData_MCP3910.p_SPITimeout = 0;
    appData_MCP3910.drvSPI1Handle= DRV_HANDLE_INVALID;
    appData_MCP3910.SPI_Buffer_Index = 0;
    appData_MCP3910.adcSamples = 0;
    appData_MCP3910.adcMeasurement = 0;
    Set1WireSpeed(1);
    p_1WireTimeOut = L_GetTickCounter();
    
    PulseCounter1 = 0 ;
    PulseCounter2 = 0;
    FrequencyIn1 = 0;
    FrequencyIn2 = 0;
    CurrentmA1 = 0;
    CurrentmA2 = 0;
    ADCCH1 = 0;
    ADCCH2 = 0;

    appData_RTCC.state =  RTCC_WAIT_FOR_SOURCE;
}
/******************************************************************************
  Function:
    void APP_Tasks_xxx ( void )

  Remarks:
    See prototype in app.h.
 */
void App_Tasks_RTC( void )
{
    switch (appData_RTCC.state)
    {
        case RTCC_WAIT_FOR_SOURCE: break;//wait till we get a correct source of date and time--either GPS or Cellular network
        case RTCC_STATE_START:

            /* Enable RTCC module */
            MySeconds = 0;
            FoundSec = false;
            PLIB_RTCC_Disable(RTCC_ID_0);
            
            appData_RTCC.state = RTCC_WAIT_CLOCk_OFF;

            break;
        case RTCC_WAIT_CLOCk_OFF:
            if(PLIB_RTCC_RTCSyncStatusGet(RTCC_ID_0)) // Wait for clock to turn off break; 
                break;
            else
                appData_RTCC.state = RTCC_SET_DATETIME;
            break;
        case RTCC_SET_DATETIME:
            //set date and time
            PLIB_RTCC_RTCYearSet(RTCC_ID_0, DEC2BCD(rtcDate.year));
            PLIB_RTCC_RTCMonthSet(RTCC_ID_0,DEC2BCD(rtcDate.month));
            PLIB_RTCC_RTCDaySet(RTCC_ID_0,DEC2BCD(rtcDate.day));
            
            PLIB_RTCC_RTCHourSet(RTCC_ID_0,DEC2BCD(rtcTime.hours));
            PLIB_RTCC_RTCMinuteSet(RTCC_ID_0,DEC2BCD(rtcTime.minutes));
            PLIB_RTCC_RTCSecondSet(RTCC_ID_0,DEC2BCD(rtcTime.seconds));
            
            PLIB_RTCC_Enable(RTCC_ID_0);
            /* Disable writes to RTCC and lock */
            PLIB_RTCC_WriteDisable(RTCC_ID_0);
            PLIB_DEVCON_SystemLock(DEVCON_ID_0);
            appData_RTCC.state = RTCC_READ_DATETIME;
            p_RTCCTimeOut = L_GetTickCounter();
            break;
        case RTCC_READ_DATETIME:
            //read RTCC
            if(L_GetTickCounter()-p_RTCCTimeOut>5) //update date time from RTC every 0.5 second
            {
                p_RTCCTimeOut = L_GetTickCounter();
                rtcDate.year = BCD2Dec(PLIB_RTCC_RTCYearGet(RTCC_ID_0));
                rtcDate.month = BCD2Dec(PLIB_RTCC_RTCMonthGet(RTCC_ID_0));
                rtcDate.day = BCD2Dec(PLIB_RTCC_RTCDayGet(RTCC_ID_0));

                rtcTime.hours = BCD2Dec(PLIB_RTCC_RTCHourGet(RTCC_ID_0));
                rtcTime.minutes = BCD2Dec(PLIB_RTCC_RTCMinuteGet(RTCC_ID_0));
                rtcTime.seconds = BCD2Dec(PLIB_RTCC_RTCSecondGet(RTCC_ID_0));
                FormatDate(true);
            }
            break;
        default:
            break;
    }
} 
void App_Tasks_General(void)
{
    if ( ADCDataReady()==true )
    {
        // get new ADC readings
        p_ADValue = calculateVoltage(0);
        if(p_ADValue<0)
            p_ADValue = 0;
        p_ADValue2 = calculateVoltage(1);
        if(p_ADValue2<0)
            p_ADValue = 0;
        ADCCH1 = p_ADValue /4096.0;//12 bits resolution
        ADCCH1 = ADCCH1 * 3.3;  //3.3V reference voltage
        ADCCH1 = ADCCH1 / 0.09; //because of the voltage divider

        ADCCH2 = p_ADValue2 /4096.0;//12 bits resolution
        ADCCH2 = ADCCH2 * 3.3;  //3.3V reference voltage
        ADCCH2 = ADCCH2 / 0.09; //because of the voltage divider
        // trigger an ADC sample
        PLIB_ADCP_GlobalSoftwareTrigger ( ADCP_ID_1 ) ;
    }
    if(L_GetTickCounter() >= p_GeneralCounter)
    {
        p_GeneralCounter = L_GetTickCounter()+10;
//        BLUE_LED_Toggle();
//        GREEN_LED_Toggle();
    }
}
void App_Tasks_ACCEL( void )
{
     switch(appData_ACCEL.state)
    {
        case APP_STATE_INIT:
        {

            appData_ACCEL.drvI2CHandle = DRV_I2C_Open( DRV_I2C_INDEX_0,
                                                     DRV_IO_INTENT_NONBLOCKING|DRV_IO_INTENT_READWRITE );
            DRV_I2C_BufferEventHandlerSet(appData_ACCEL.drvI2CHandle, APP_I2C_MasterOpEndCallback, operationStatus );
            appData_ACCEL.state = ACCEL_RD_WHOAMI;
            p_AccelTimeout = L_GetTickCounter();
            break;
        }
         case ACCEL_RD_WHOAMI:
         {
             if(L_GetTickCounter()-p_AccelTimeout >5)
             {
                 //appData_ACCEL.state = ACCEL_STDBY ;
                p_AccelTimeout = L_GetTickCounter();
                AccelAddress = FXOS8700CQ_SLAVE_ADDR;
                appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_WHOAMI;
                DRV_I2C_BufferAddWriteRead(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 1, (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CRXbuffer[0], 1, NULL);
             }
             break;
         }
         case ACCEL_STDBY:
         {
             if(L_GetTickCounter()-p_AccelTimeout >0)
             {
                 //appData_ACCEL.state = ACCEL_CONFIGMAG_REG1 ;
                p_AccelTimeout = L_GetTickCounter();
                 appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_CTRL_REG1;
                 appData_ACCEL.drvI2CTXbuffer[1] = 0b00000000;
                 DRV_I2C_BufferAddWrite(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 2, NULL);
             }
         }
         break;
         case ACCEL_CONFIGMAG_REG1:
         {
             if(L_GetTickCounter()-p_AccelTimeout >0)
             {
                 //appData_ACCEL.state = ACCEL_CONFIGMAG_REG2 ;
                 p_AccelTimeout = L_GetTickCounter();
                appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_M_CTRL_REG1;
                appData_ACCEL.drvI2CTXbuffer[1] = 0x1F;
                DRV_I2C_BufferAddWrite(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 2, NULL);
             }
         }
         break;
         case ACCEL_CONFIGMAG_REG2:
         {
             if(L_GetTickCounter()-p_AccelTimeout >0)
             {
                 //appData_ACCEL.state = ACCEL_XYZ_CONFIG ;
                 p_AccelTimeout = L_GetTickCounter();
                appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_M_CTRL_REG2;
                appData_ACCEL.drvI2CTXbuffer[1] = 0x20;
                DRV_I2C_BufferAddWrite(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 2, NULL);
             }
         }
         break;
         case ACCEL_XYZ_CONFIG:
         {
             if(L_GetTickCounter()-p_AccelTimeout >0)
             {
                 //appData_ACCEL.state = ACCEL_CONFIG_REG1 ;
                 p_AccelTimeout = L_GetTickCounter();
                appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_XYZ_DATA_CFG;
                appData_ACCEL.drvI2CTXbuffer[1] = 0x01;
                DRV_I2C_BufferAddWrite(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 2, NULL);
             }
         }
         break;
         case ACCEL_CONFIG_REG1:
         {
             if(L_GetTickCounter()-p_AccelTimeout >0)
             {
                //appData_ACCEL.state = ACCEL_ReadData ;
                p_AccelTimeout = L_GetTickCounter();
                appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_CTRL_REG1;
                appData_ACCEL.drvI2CTXbuffer[1] = 0x0D;
                DRV_I2C_BufferAddWrite(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 2, NULL);
             }
         }
         break;
         case ACCEL_ReadData:
         {
             //ACCELDATA
             if(L_GetTickCounter()-p_AccelTimeout >10)
             {
                 if(AccelDataReady==true)
                {
                    accelData.x = (int16_t)(((appData_ACCEL.drvI2CRXbuffer[1] << 8) | appData_ACCEL.drvI2CRXbuffer[2]))>> 2;
                    accelData.y = (int16_t)(((appData_ACCEL.drvI2CRXbuffer[3] << 8) | appData_ACCEL.drvI2CRXbuffer[4]))>> 2;
                    accelData.z = (int16_t)(((appData_ACCEL.drvI2CRXbuffer[5] << 8) | appData_ACCEL.drvI2CRXbuffer[6]))>> 2;
                    accelData.VectorData  = accelData.x * accelData.x;
                    accelData.VectorData += (accelData.y * accelData.y);
                    accelData.VectorData += (accelData.z * accelData.z);
                    accelData.VectorData = sqrt(accelData.VectorData);
                    // copy the magnetometer byte data into 16 bit words
                    magData.x = (appData_ACCEL.drvI2CRXbuffer[7] << 8) | appData_ACCEL.drvI2CRXbuffer[8];
                    magData.y = (appData_ACCEL.drvI2CRXbuffer[9] << 8) | appData_ACCEL.drvI2CRXbuffer[10];
                    magData.z = (appData_ACCEL.drvI2CRXbuffer[11] << 8) | appData_ACCEL.drvI2CRXbuffer[12];
                    magData.VectorData  = magData.x * magData.x;
                    magData.VectorData += (magData.y * magData.y);
                    magData.VectorData += (magData.z * magData.z);
                    magData.VectorData = sqrt(magData.VectorData);
//                    L_CopyToArray(&p_USBDebugMessage[0],"Accel=");
//                    p_USBDebugMessageSize=6;
//                    p_USBDebugMessageSize += IntToStr(accelData.VectorData,&p_USBDebugMessage[p_USBDebugMessageSize]);
//                    L_CopyToArray(&p_USBDebugMessage[p_USBDebugMessageSize],";Magn=");
//                    p_USBDebugMessageSize+=6;
//                    p_USBDebugMessageSize += IntToStr(magData.VectorData,&p_USBDebugMessage[p_USBDebugMessageSize]);
//                    p_USBDebugMessage[p_USBDebugMessageSize] = '\r';
//                    p_USBDebugMessageSize++ ;
//                    p_USBDebugMessage[p_USBDebugMessageSize] = '\n';
//                    p_USBDebugMessageSize++;
//                    p_DebugMessagesCount = 1;
                    AccelDataReady = false;
                }
                appData_ACCEL.drvI2CTXbuffer[0] = FXOS8700CQ_STATUS;
                DRV_I2C_BufferAddWriteRead(appData_ACCEL.drvI2CHandle, &AccelAddress , (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CTXbuffer[0], 1, (I2C_DATA_TYPE *)&appData_ACCEL.drvI2CRXbuffer[0], 13, NULL);
                p_AccelTimeout = L_GetTickCounter();
             }
         }
             break;
     }
}
void APP_Tasks_MCP3910( void )
{
    switch(appData_MCP3910.state)
    {
        case APP_STATE_INIT:
        {
            /* Open the SPI1 Driver */
            if(appData_MCP3910.drvSPI1Handle == DRV_HANDLE_INVALID)
            {
                appData_MCP3910.drvSPI1Handle = DRV_SPI_Open( DRV_SPI_INDEX_0,
                                                      DRV_IO_INTENT_READWRITE |
                                                      DRV_IO_INTENT_NONBLOCKING);
            }

            if((appData_MCP3910.drvSPI1Handle != DRV_HANDLE_INVALID))
            {
                appData_MCP3910.state = APP_STATE_CHECK_DRVR_STATE;
            }
        }
        break;

        case APP_STATE_CHECK_DRVR_STATE:
        {
            /* Check the SPI1 driver handler */
            if (appData_MCP3910.drvSPI1Handle == DRV_HANDLE_INVALID )
            {
                /* Set the app state to initialization */
                appData_MCP3910.state   = APP_STATE_INIT;
                return;
            }
             
            // this is the ~RESET pin - when logic low, no communication 
            PLIB_PORTS_PinClear(ADC_Select);
                       
            appData_MCP3910.state = APP_STATE_CONFIG_1;
        }
        break;
        
        case APP_STATE_CONFIG_1:
        {   
            
            // Config1 register address - to be written to
            appData_MCP3910.drvSPI1TXbuffer[0]=0x5C; 
            
            // Configuration for Config1
            appData_MCP3910.drvSPI1TXbuffer[1]=0x00; 
            appData_MCP3910.drvSPI1TXbuffer[2]=0x00;  
            appData_MCP3910.drvSPI1TXbuffer[3]=0x00;
            
            appData_MCP3910.gBufferContext.SPI1TxBufferRef = &appData_MCP3910.drvSPI1TXbuffer[0];
            
            // important to change the state before adding to write/read buffer
            appData_MCP3910.state = APP_STATE_WAIT_CONFIG1;
            
            PLIB_PORTS_PinClear(ADC_CS);
            
            appData_MCP3910.drvSPI1TxBufHandle = DRV_SPI_BufferAddWrite(
                                appData_MCP3910.drvSPI1Handle,
                                  (uint8_t *)&appData_MCP3910.drvSPI1TXbuffer[0],
                                    4, APP_BufferEventHandlerSPI1,
                                      appData_MCP3910.gBufferContext.SPI1TxBufferRef );
            
        }
        break;
       
        case APP_STATE_WAIT_CONFIG1:
        {
            
            Nop();
        }
        break;
        
        case APP_STATE_READ_CH0:
        {   
            // Channel 0 data register
            appData_MCP3910.drvSPI1TXbuffer[0]=0x41;
            
            // Initialize Rx before starting a read operation
            ADC_ClearSPIRXBuffer();
            
            appData_MCP3910.SPI_Buffer_Index = 0;
            
            appData_MCP3910.gBufferContext.SPI1TxBufferRef = &appData_MCP3910.drvSPI1TXbuffer[0];
            
            appData_MCP3910.gBufferContext.SPI1RxBufferRef = &appData_MCP3910.drvSPI1RXbuffer[0];
                       
            appData_MCP3910.state = APP_STATE_WAIT_READ_COMPLETE;
            
            PLIB_PORTS_PinClear(ADC_CS);
            appData_MCP3910.drvSPI1TxBufHandle = DRV_SPI_BufferAddWrite(
                            appData_MCP3910.drvSPI1Handle,
                                (uint8_t *)&appData_MCP3910.drvSPI1TXbuffer[0],
                                    1, APP_BufferEventHandlerSPI1,
                                        appData_MCP3910.gBufferContext.SPI1TxBufferRef );
            
            
            appData_MCP3910.drvSPI1RxBufHandle = DRV_SPI_BufferAddRead(
                            appData_MCP3910.drvSPI1Handle,
                                (uint8_t *)&appData_MCP3910.drvSPI1RXbuffer[0],
                                    3, APP_BufferEventHandlerSPI1,
                                        appData_MCP3910.gBufferContext.SPI1RxBufferRef );
            
            //PLIB_PORTS_PinSet(ADC_CS);
            
               
        }
        break;
                
        case APP_STATE_WAIT_READ_COMPLETE:
        {
            
            Nop();
        }
        break;
        
        case APP_STATE_READ_DONE:
        {
            if(L_GetTickCounter()- appData_MCP3910.p_SPITimeout>10)
            {
                
                // have to write the Config1 again before reading the next measurement
                appData_MCP3910.state = APP_STATE_CONFIG_1;
                
                // joining the 3 bytes into one 3-byte chunk
                appData_MCP3910.adcSamples = (appData_MCP3910.drvSPI1RXbuffer[0] << 16) |
                                            (appData_MCP3910.drvSPI1RXbuffer[1] << 8)  |
                                               (appData_MCP3910.drvSPI1RXbuffer[2]) ;
                appData_MCP3910.adcMeasurement = (((appData_MCP3910.adcSamples * (MCP3910_VREF)) / (12582912.f))/MCP3910_Resistor)*1000;  
                p_USBDebugMessageSize = 0;
                L_CopyToArray(&p_USBDebugMessage[p_USBDebugMessageSize],"Current=");
                p_USBDebugMessageSize+=8;
                p_USBDebugMessageSize += ftoa2(appData_MCP3910.adcMeasurement,&p_USBDebugMessage[p_USBDebugMessageSize]);
                p_USBDebugMessage[p_USBDebugMessageSize] = 'm';
                p_USBDebugMessageSize++ ;
                p_USBDebugMessage[p_USBDebugMessageSize] = 'A';
                p_USBDebugMessageSize++ ;
                p_USBDebugMessage[p_USBDebugMessageSize] = '\r';
                p_USBDebugMessageSize++ ;
                p_USBDebugMessage[p_USBDebugMessageSize] = '\n';
                p_USBDebugMessageSize++;
                p_DebugMessagesCount = 1;
                ADC_ClearSPIRXBuffer();
                
            }

        }
        break;
        
        case APP_STATE_RESET:
        {
               
            appData_MCP3910.drvSPI1TXbuffer[0]=0x5C; 
            
            // This configuration puts ADC in shutdown mode
            appData_MCP3910.drvSPI1TXbuffer[1]=0x00; 
            appData_MCP3910.drvSPI1TXbuffer[2]=0x03;  
            appData_MCP3910.drvSPI1TXbuffer[3]=0xC0;
            
            appData_MCP3910.gBufferContext.SPI1TxBufferRef = &appData_MCP3910.drvSPI1TXbuffer[0];
            
            appData_MCP3910.state = APP_STATE_RESUME;
            PLIB_PORTS_PinClear(ADC_CS);
            appData_MCP3910.drvSPI1TxBufHandle = DRV_SPI_BufferAddWrite(
                            appData_MCP3910.drvSPI1Handle,
                                (uint8_t *)&appData_MCP3910.drvSPI1TXbuffer[0],
                                    4, APP_BufferEventHandlerSPI1,
                                        appData_MCP3910.gBufferContext.SPI1TxBufferRef );
            
        }
        break;
        
        case APP_STATE_RESUME:
        {   
             // Wait before resuming the normal operation
             if(L_GetTickCounter()- appData_MCP3910.p_SPITimeout>20)
             {
                 
                appData_MCP3910.state = APP_STATE_CONFIG_1;
             }
            
        }
        break;
        
        default:
            break;
    }
}
void App_Task_1Wire( void )
{
    uint8_t TempH;
    uint8_t TempL;
    int16_t Temperature1Wire;
    if(L_GetTickCounter()-p_1WireTimeOut >10)
    {
        OWTouchReset();
        OWWriteByte(0xCC);  //skip ROM
        OWWriteByte(0x44);  //Start Conversion
        ShortDelay(100);
        OWTouchReset();
        OWWriteByte(0xCC);  //Skip ROM
        OWWriteByte(0xBE);  //Read Scratch PAD
        TempL = OWReadByte();
        TempH = OWReadByte();
        Temperature1Wire = TempH<<8;
        Temperature1Wire += TempL;
        Temperature1Wire = Temperature1Wire & 0xFFF0;
        Temperature1Wire = Temperature1Wire >>4;        
        p_1WireTimeOut = L_GetTickCounter(); 
    }
}
void App_Task_Flash( void )
{
    SQI_STATUS sqiStatus;
    if(!PLIB_SQI_ClockIsStable(SQI_ID_0)) //wait for clock to stabilize
        return;
    PLIB_PORTS_PinClear(SQI_PWR);
    if(L_GetTickCounter()<50)
        return;
    switch ( appData_Flash.state )
    {
        case APP_STATE_INIT:
        {
            SQI_Flash_Setup();
            appData_Flash.state = APP_STATE_FLASH_ID_READ;
        }break;
        case APP_STATE_FLASH_ID_READ:
        {
            /* Get the ID read status */
            sqiStatus = SQI_FlashID_Read();
            if ( sqiStatus !=  SQI_STATUS_SUCCESS)
                appData_Flash.state = APP_STATE_FLASH_ID_READ;
            else
            {
                SendUSBDebugMessage("Memory test is successful");
                appData_Flash.state = APP_STATE_WRITE_FLASH;
            }
            break;

        }

        /* Write flash*/
        case APP_STATE_WRITE_FLASH:
        {
            SQI_PIO_PageWrite(FLASH_PAGE_ADDR);

            /* Update the state */
            appData_Flash.state = APP_STATE_READ_FLASH_DMA_MODE;
        }

        /* Read flash ID until successful */
        case APP_STATE_READ_FLASH_DMA_MODE:
        {
            /* Get the ID read status */
            sqiStatus = SQI_DMA_Read(FLASH_PAGE_ADDR);
            if ( sqiStatus !=  SQI_STATUS_SUCCESS)
                /* Update the state */
                appData_Flash.state = APP_STATE_INIT;
            else
                /* Update the state */
                appData_Flash.state = APP_STATE_DONE;
            break;

        }
        /* Idle state (do nothing) */
        case APP_STATE_DONE:break;
        default:
            
            break;
    }
}
void App_Tasks_IC()
{
//    switch(appDataIC.state)
//    {
//        case APP_STATE_INIT:
//            DRV_IC0_Start();
//            appDataIC.state = APP_STATE_INIT;
//            break;
//        case APP_STATE_SCHEDULE_READ:
//            if(DRV_IC0_BufferIsEmpty()==false)
//            {
//                DRV_IC0_Capture32BitDataRead();
//                L_CopyToArray(&p_USBDebugMessage[0],"Timer=");
//                p_USBDebugMessageSize=6;
//                p_USBDebugMessageSize += IntToStr(DRV_IC0_Capture32BitDataRead(),&p_USBDebugMessage[p_USBDebugMessageSize]);
//                p_USBDebugMessage[p_USBDebugMessageSize] = '\r';
//                p_USBDebugMessageSize++ ;
//                p_USBDebugMessage[p_USBDebugMessageSize] = '\n';
//                p_USBDebugMessageSize++;
//                p_DebugMessagesCount = 1;
//                DRV_IC0_Stop();
//            }
//            break;
//    }
}
void App_Task_XE910 ( void )
{
   /* Check the XE910's current state. */
    //ClearWDT();
    size_t p_MessageSize=0;

    if(U1STAbits.OERR==1)
        U1STAbits.OERR=0;
    
    XE910_CheckPower();
    switch ( appData_XE910.XE910_State )
    {
        /* Application's initial state. */
        case XE910_POWER_ON:
        {
            if(appData_XE910.drvXE910UsartHandle == DRV_HANDLE_INVALID)
            {
            	/* Open the USART Driver for USART1 Client  */
		appData_XE910.drvXE910UsartHandle = DRV_USART_Open( DRV_USART_INDEX_1,
                                                            DRV_IO_INTENT_NONBLOCKING|
							 DRV_IO_INTENT_READWRITE );
            }
            PLIB_PORTS_PinClear(XE910_SHTDN);
            PLIB_PORTS_PinClear(SIM_SELECTTION);
            PLIB_PORTS_PinSet(XE910_OE);
            PLIB_PORTS_PinSet(XE910_Switch);
            PLIB_PORTS_PinSet(XE910_ON_OFF);
            if(XE910_PWRMON()!=0)
            {
                //Module is already on, so skip to the right step
                appData_XE910.XE910_State = XE910_SETUP_USART;
                return;
            }

            BLUE_LED_ON();
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_WAIT_TILL_ON;
            break;
        }
        case XE910_WAIT_TILL_ON:
        {
            if((L_GetTickCounter()- p_XE910Timeout)>55)
            {
                SendUSBDebugMessage("Module is on!!");
                BLUE_LED_OFF();
                PLIB_PORTS_PinClear(XE910_ON_OFF);
                appData_XE910.XE910_State = XE910_SETUP_USART;
            }
            break;
        }
        case XE910_TIMEOUT_WAIT:
        {
            p_MessageSize = DRV_USART_Read(appData_XE910.drvXE910UsartHandle,(uint8_t*)&appData_XE910.drvXE910UsartRxBuffer[appData_XE910.XE910_Buffer_Index],100);
            appData_XE910.XE910_Buffer_Index += p_MessageSize;
            if((L_GetTickCounter()- p_XE910Timeout)>10 )
            {
                if(appData_XE910.XE910ResponseState==AT_SGACT_Q_RESPONSE)
                {
                    if(XE910_CheckContext()==true)//
                    {
                        /*Context ia already active - move to AT#SD command*/
                        appData_XE910.XE910_State = AT_SD_SEND;
                        return;
                    }
                }

                if(XE910_CheckForOK()==true)
                {
                    switch(appData_XE910.XE910ResponseState)
                    {
                        case AT_RESPONSE: appData_XE910.XE910_State = AT_GPIO_SEND;   break;
                        case AT_GPIO_RESPONSE: appData_XE910.XE910_State = AT_CMEE_SEND;   break;
                        case AT_CMEE_RESPONSE: appData_XE910.XE910_State = AT_CPIN_SEND;   break;
                        case AT_CPIN_RESPONSE: appData_XE910.XE910_State = AT_CGSN_SEND;   break;
                        case AT_CGSN_RESPONSE: appData_XE910.XE910_State = AT_ANDC1_SEND;   break;
                        case AT_ANDC1_RESPONSE: appData_XE910.XE910_State = AT_SELINT_SEND;   break;
                        case AT_SELINT_RESPONSE: appData_XE910.XE910_State = AT_SCFG_SEND;   break;
                        case AT_SCFG_RESPONSE: appData_XE910.XE910_State = AT_CGDCONT_SEND;   break;
                        case AT_CGDCONT_RESPONSE:  appData_XE910.XE910_State = AT_SGACT_Q_SEND;   break;
                        case AT_SGACT_Q_RESPONSE:appData_XE910.XE910_State = AT_SGACT_SEND;    break;
                        case AT_SGACT_RESPONSE: appData_XE910.XE910_State = AT_CCLK_SEND;   break;
                        case AT_CCLK_RESPONSE: appData_XE910.XE910_State = AT_MONI_SEND;   break;
                        case AT_MONI_RESPONSE: appData_XE910.XE910_State = AT_SD_SEND;   break;
                        case AT_SD_RESPONSE: appData_XE910.XE910_State = XE910_SEND_DATA;    break;
                        default: break;
                    }
                }
            }
            if((L_GetTickCounter()- p_XE910Timeout)>50 )//timeout occurred with no feedback
            {
                SendUSBDebugMessage("Command Time out");
                switch(appData_XE910.XE910ResponseState)
                {
                    case AT_RESPONSE: appData_XE910.XE910_State = AT_GPIO_SEND;   break;
                    case AT_GPIO_RESPONSE: appData_XE910.XE910_State = AT_CMEE_SEND;   break;//move forward anyway since some module return an error for this command
                    case AT_CMEE_RESPONSE: appData_XE910.XE910_State = AT_CMEE_SEND;   break;
                    case AT_CPIN_RESPONSE: appData_XE910.XE910_State = AT_CPIN_SEND;   break;
                    case AT_CGSN_RESPONSE: appData_XE910.XE910_State = AT_CGSN_SEND;   break;
                    case AT_ANDC1_RESPONSE: appData_XE910.XE910_State = AT_ANDC1_SEND;   break;
                    case AT_CCLK_RESPONSE: appData_XE910.XE910_State = AT_CCLK_SEND;   break;
                    case AT_SELINT_RESPONSE: appData_XE910.XE910_State = AT_SELINT_SEND;   break;
                    case AT_SCFG_RESPONSE: appData_XE910.XE910_State = AT_SCFG_SEND;   break;
                    case AT_CGDCONT_RESPONSE:  appData_XE910.XE910_State = AT_CGDCONT_SEND;   break;
                    case AT_SGACT_Q_RESPONSE:appData_XE910.XE910_State = AT_SGACT_Q_SEND;    break;
                    case AT_SGACT_RESPONSE: appData_XE910.XE910_State = AT_SGACT_SEND;   break;
                    case AT_MONI_RESPONSE: appData_XE910.XE910_State = AT_MONI_SEND;   break;
                    case AT_SD_RESPONSE: appData_XE910.XE910_State = XE910_SEND_DATA;    break;
                    default: break;
                }
            }
            break;
        }
        case XE910_SETUP_USART:
        {
            DRV_USART_BufferEventHandlerSet(appData_XE910.drvXE910UsartHandle,APP_BufferEventHandlerXE910Usart,(uintptr_t)1);
            appData_XE910.XE910_State = AT_SEND;

            break;
        }
        case AT_SEND:
        {   
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], 3);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_RESPONSE;
            break;
        }
        case AT_GPIO_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#GPIO=1,0,2\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_GPIO_RESPONSE ;
            break;
        }
        case AT_CMEE_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT+CMEE=2\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_CMEE_RESPONSE ;
            break;
        }
        case AT_CPIN_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT+CPIN?\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_CPIN_RESPONSE ;
            break;
        }
        case AT_CGSN_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT+CGSN\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_CGSN_RESPONSE ;
            break;
        }
        case AT_CCLK_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT+CCLK?\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_CCLK_RESPONSE ;
            break;
        }
        case AT_ANDC1_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT&C1\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_ANDC1_RESPONSE ;
            break;
        }
        case AT_SELINT_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#SELINT=2\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_SELINT_RESPONSE ;
            break;
        }
        case AT_SCFG_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#SCFG=1,1,300,90,600,50\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_SCFG_RESPONSE ;
            break;
        }
        case AT_CGDCONT_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT+CGDCONT=1,\"IP\",\"internetm2m.air.com\",\"0.0.0.0\",0,0\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_CGDCONT_RESPONSE;
            break;
        }
        case AT_SGACT_Q_SEND:
        {
            if(XE910_IsNotConnected()!=1)
            {
                //socket is open. need to close it and then proceed
                XE910_ClearXE910RXBuffer();
                p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"+++");
                DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
                appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
                appData_XE910.XE910ResponseState = AT_SGACT_Q_RESPONSE  ;
            }
            else
            {
                XE910_ClearXE910RXBuffer();
                p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#SGACT?\r");
                DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
                p_XE910Timeout = L_GetTickCounter();
                appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
                appData_XE910.XE910ResponseState = AT_SGACT_Q_RESPONSE  ;
            }
            break;
        }
        case AT_SGACT_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#SGACT=1,1,\"\",\"\"\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_SGACT_RESPONSE ;
            break;
        }
        case AT_MONI_SEND:
        {
            XE910_ClearXE910RXBuffer();
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#MONI\r");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.XE910_State = XE910_TIMEOUT_WAIT;
            appData_XE910.XE910ResponseState = AT_MONI_RESPONSE ;
            break;
        }
        case AT_SD_SEND:
        {
            fixQuality = 1;
            if(fixQuality==0)
            {
                appData_XE910.TxIntervalWait = true;
                appData_XE910.XE910_State = XE910_Delay;
                //wait till we get a fix
                p_XE910Timeout = L_GetTickCounter();
                SendUSBDebugMessage("GPS doesn't have a fix. Try again in 10 seconds");
            }
            else
            {
                //SendUSBDebugMessage("GPS has a fix. Connecting to Platform");
                //XE910_ClearXE910RXBuffer();
                //appData_XE910.XE910_Buffer_Index = 0;
                p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"AT#SD=1,0,80,\"rs2.cumulocity.com\",0,0\r");
                DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
                p_XE910Timeout = L_GetTickCounter();
                appData_XE910.XE910_State = XE910_Delay;
                appData_XE910.XE910ResponseState = AT_SD_RESPONSE ;
                appData_XE910.TxIntervalWait = false;
            }
            break;
        }
        case XE910_SEND_DATA:
        {
            if(XE910_IsNotConnected()==1)
            {
                appData_XE910.XE910_State = AT_SGACT_Q_SEND;  //go back to state to check context
                //SendUSBDebugMessage("TCP connection is not established");
                return;
            }

            BLUE_LED_ON();
            
            //XE910_ClearXE910RXBuffer();
            //BuildJSONPost();  //to Send IO data
            BuildJSONPost_Accel();// To send accelerometer and magnetometer data
            p_MessageSize = L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[0],"POST /measurement/measurements HTTP/1.1\r\nHost: rs2.cumulocity.com\r\nAuthorization: Basic xxxxx=\r\nContent-Type: application/vnd.com.nsn.cumulocity.measurement+json\r\nCache-Control: no-cache\r\nConnection: keep-alive\r\nContent-Length: ");
            p_MessageSize+= IntToStr(appData_XE910.XE910_CumulocityBufferIndex,&appData_XE910.drvXE910UsartTxBuffer[p_MessageSize]);
            p_MessageSize += L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[p_MessageSize],"\r\n\r\n");
            L_CopyArray1ToArray2(&appData_XE910.XE910_CumulocityBuffer[0],&appData_XE910.drvXE910UsartTxBuffer[p_MessageSize],appData_XE910.XE910_CumulocityBufferIndex);
            p_MessageSize += appData_XE910.XE910_CumulocityBufferIndex;


            //p_MessageSize += L_CopyToArray(&appData_XE910.drvXE910UsartTxBuffer[p_MessageSize],",\"unit\":\"mA\"}}}");
            DRV_USART_BufferAddWrite(appData_XE910.drvXE910UsartHandle,&(appData_XE910.drvXE910UsartTxBufHandle),(uint8_t *)&appData_XE910.drvXE910UsartTxBuffer[0], p_MessageSize);
            p_XE910Timeout = L_GetTickCounter();
            appData_XE910.TxIntervalWait = true;
            appData_XE910.XE910_State = XE910_Delay;
            appData_XE910.XE910ResponseState = AT_SD_RESPONSE ;
            //SendUSBDebugMessage("Sending a post message");
            break;
        }
        case XE910_Delay:
            if(appData_XE910.TxIntervalWait==false) //wait to get connect from the telit module
            {
                p_MessageSize = DRV_USART_Read(appData_XE910.drvXE910UsartHandle,(uint8_t*)&appData_XE910.drvXE910UsartRxBuffer[appData_XE910.XE910_Buffer_Index],100);
                appData_XE910.XE910_Buffer_Index += p_MessageSize;
                if((L_GetTickCounter()- p_XE910Timeout)>20 && XE910_IsNotConnected()!=1)
                {
                    XE910_SendUART2USB();
                    appData_XE910.XE910_Buffer_Index =0;
                    appData_XE910.XE910_State = XE910_SEND_DATA;
                }
                if((L_GetTickCounter()- p_XE910Timeout)>80)
                {
                    XE910_SendUART2USB();
                    appData_XE910.XE910_Buffer_Index  = 0;
                    appData_XE910.XE910_State = XE910_SEND_DATA;
                }
            }
            else
            {
                p_MessageSize = DRV_USART_Read(appData_XE910.drvXE910UsartHandle,(uint8_t*)&appData_XE910.drvXE910UsartRxBuffer[appData_XE910.XE910_Buffer_Index],250);
                appData_XE910.XE910_Buffer_Index += p_MessageSize;
                if((L_GetTickCounter()- p_XE910Timeout)>10 )
                {
                    XE910_SendUART2USB();
                    appData_XE910.XE910_Buffer_Index  = 0;
                    appData_XE910.XE910_State = XE910_SEND_DATA;
                }
            }
                    
            break;
        case XE910_SET_GPIO:
        {
            if(XE910_PWRMON()!=0)
            {
                BLUE_LED_ON();
                
            }
            
            appData_XE910.XE910_State = APP_STATE_NOTHING;
            break;
        }
        case APP_STATE_NOTHING:
        {
            if(U1STAbits.OERR==1)
                U1STAbits.OERR=0;
             if(XE910_PWRMON()!=0)
            {
                BLUE_LED_ON();
                
            }
            break;
        }
        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
void App_Tasks_GPS(void)
{
    switch ( appData_GPS.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            BLUE_LED_OFF();
            
            if(appData_GPS.drvUsart2Handle == DRV_HANDLE_INVALID)
            {
            	/* Open the USART Driver for USART2 Client  */
                appData_GPS.drvUsart2Handle = DRV_USART_Open( DRV_USART_INDEX_0,
                                      //DRV_IO_INTENT_NONBLOCKING|
                                     DRV_IO_INTENT_READWRITE );
            }
            appData_GPS.state = APP_STATE_CHECK_DRVR_STATE;
            break;
        }
        case APP_STATE_CHECK_DRVR_STATE:
         {
            /* Check the USART2 driver handler */
            if (appData_GPS.drvUsart2Handle == DRV_HANDLE_INVALID )
            {
                /* Set the app state to Ready */
                appData_GPS.state   = APP_STATE_INIT;
                return;
            }
            BLUE_LED_ON();
            DRV_USART_BufferEventHandlerSet(appData_GPS.drvUsart2Handle,APP_BufferEventHandlerUsart2,(uintptr_t)2);
            appData_GPS.state = APP_STATE_GetGPSData;
            DRV_USART_BufferAddRead(appData_GPS.drvUsart2Handle,&(appData_GPS.drvUsart2RxBufHandle),
            (uint8_t*)&appData_GPS.drvUsart2RxBuffer[0],512);

	}
        break;


        case APP_STATE_GetGPSData:
        {

            if(U2STAbits.OERR==1)
                    U2STAbits.OERR=0;
            
            if(appData_GPS.drvUsart2RxBuffer_Index>0)
            {
                appData_GPS.drvUsart2RxBuffer_Index = 0;
                DRV_USART_BufferAddRead(appData_GPS.drvUsart2Handle,&(appData_GPS.drvUsart2RxBufHandle),
                        (uint8_t*)&appData_GPS.drvUsart2RxBuffer[0],512);
                
                if(L_GetTickCounter()>=p_GPSTimeout)
                {
                    //Send to USB
                    BLUE_LED_Toggle();
                    p_GPSTimeout = L_GetTickCounter()+1;
                    //GPS_SendUART2USB();
                    
                    
                }
                
            }

            

        }break;
        /* TODO: implement your application state machine.*/

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}
void APP_Tasks_USB ( void )
{
    USB_DEVICE_CDC_RESULT result;
    switch(appData_USB.state)
    {
        case APP_STATE_INIT:

            /* Open the device layer */
            appData_USB.deviceHandle = USB_DEVICE_Open( USB_DEVICE_INDEX_0, DRV_IO_INTENT_READWRITE );

            if(appData_USB.deviceHandle != USB_DEVICE_HANDLE_INVALID)
            {
                /* Register a callback with device layer to get event notification (for end point 0) */
                USB_DEVICE_EventHandlerSet(appData_USB.deviceHandle, APP_USBDeviceEventHandler, 0);

                appData_USB.state = APP_STATE_WAIT_FOR_CONFIGURATION;
            }
            else
            {
                /* The Device Layer is not ready to be opened. We should try
                 * again later. */
            }

            break;

        case APP_STATE_WAIT_FOR_CONFIGURATION:

            /* Check if the device was configured */
            if(appData_USB.isConfigured)
            {
                /* If the device is configured then lets start reading */
                appData_USB.state = APP_STATE_SCHEDULE_READ;
            }
            break;

        case APP_STATE_SCHEDULE_READ:

            if(APP_StateReset())
            {
                break;
            }

            /* If a read is complete, then schedule a read
             * else wait for the current read to complete */

            appData_USB.state = APP_STATE_WAIT_FOR_READ_COMPLETE;
            if(appData_USB.isReadComplete == true)
            {
                appData_USB.isReadComplete = false;
                appData_USB.readTransferHandle =  USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;

                USB_DEVICE_CDC_Read (USB_DEVICE_CDC_INDEX_0,
                        &appData_USB.readTransferHandle, appData_USB.readBuffer,
                        APP_READ_BUFFER_SIZE);
                
                if(appData_USB.readTransferHandle == USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID)
                {
                    appData_USB.state = APP_STATE_ERROR;
                    break;
                }
            }
            break;

        case APP_STATE_WAIT_FOR_READ_COMPLETE:

            if(APP_StateReset())
            {
                break;
            }

            if(p_DebugMessagesCount>0)
            {
                appData_USB.state = APP_STATE_SCHEDULE_WRITE;
                p_DebugMessagesCount=0;
            }
             /* The isReadComplete flag gets updated in the CDC event handler. */

            if(appData_USB.isReadComplete)
            {
                //Process received data
                //echo back the received messages
                p_USBDebugMessageSize = 0;
                while(p_USBDebugMessageSize<RXDatalength)
                {
                    p_USBDebugMessage[p_USBDebugMessageSize] = appData_USB.readBuffer[p_USBDebugMessageSize];
                    p_USBDebugMessageSize++;
                    p_DebugMessagesCount =1;
                }
                RXDatalength = 0;
                
                appData_USB.state = APP_STATE_SCHEDULE_READ;
            }

            break;


        case APP_STATE_SCHEDULE_WRITE:

            if(APP_StateReset())
            {
                break;
            }

            /* Setup the write */

            appData_USB.writeTransferHandle = USB_DEVICE_CDC_TRANSFER_HANDLE_INVALID;
            appData_USB.isWriteComplete = false;
            appData_USB.state = APP_STATE_WAIT_FOR_WRITE_COMPLETE;

            result = USB_DEVICE_CDC_Write(USB_DEVICE_CDC_INDEX_0,
                    &appData_USB.writeTransferHandle,  p_USBDebugMessage,p_USBDebugMessageSize,
                    USB_DEVICE_CDC_TRANSFER_FLAGS_DATA_COMPLETE);
                    p_DebugMessagesCount=0;
            break;

        case APP_STATE_WAIT_FOR_WRITE_COMPLETE:

            if(APP_StateReset())
            {
                break;
            }

            /* Check if a character was sent. The isWriteComplete
             * flag gets updated in the CDC event handler */

            if(appData_USB.isWriteComplete == true)
            {
                appData_USB.state = APP_STATE_SCHEDULE_READ;
            }

            break;

        case APP_STATE_ERROR:
            break;
        default:
            break;
    }
}
/***********Custom Functions*************/
uint8_t GetIOStatus(uint8_t PortNumber)
{
    switch(PortNumber)
    {
        case 1:if(GetPin(DI1P)==false && GetPin(DI1G)==true) return FLOATING;if(GetPin(DI1P)==true && GetPin(DI1G)==true) return LOGIC_LOW;if(GetPin(DI1P)==false && GetPin(DI1G)==false) return LOGIC_HIGH; break;
        case 2:if(GetPin(DI2P)==false && GetPin(DI2G)==true) return FLOATING;if(GetPin(DI2P)==true && GetPin(DI2G)==true) return LOGIC_LOW;if(GetPin(DI2P)==false && GetPin(DI2G)==false) return LOGIC_HIGH; break;
        case 3:if(GetPin(DI3P)==false && GetPin(DI3G)==true) return FLOATING;if(GetPin(DI3P)==true && GetPin(DI3G)==true) return LOGIC_LOW;if(GetPin(DI3P)==false && GetPin(DI3G)==false) return LOGIC_HIGH; break;
        case 4:if(GetPin(DI4P)==false && GetPin(DI4G)==true) return FLOATING;if(GetPin(DI4P)==true && GetPin(DI4G)==true) return LOGIC_LOW;if(GetPin(DI4P)==false && GetPin(DI4G)==false) return LOGIC_HIGH; break;
    }
    return FLOATING; //unrecognized port number or invalid status, so we return floating as status is unknown
}
void SendUSBDebugMessage(const char* Src)
{
    p_USBDebugMessageSize=0;
    while(*Src)
    {
        p_USBDebugMessage[p_USBDebugMessageSize] = *Src++;
        p_USBDebugMessageSize++ ;
    }
    p_USBDebugMessage[p_USBDebugMessageSize] = '\r';
    p_USBDebugMessageSize++ ;
    p_USBDebugMessage[p_USBDebugMessageSize] = '\n';
    p_USBDebugMessageSize++;

    p_DebugMessagesCount=1;
}
void Timer_CounterCallback(uintptr_t context, uint32_t alarmCount)
{
    p_TickCounter++;
    //compute Frequencies here
    FrequencyIn1 = PulseCounter1 * 5;  // divide by 2 and multiply by 10 since this is triggered every 0.1 second
    FrequencyIn2 = PulseCounter2 * 5;
    
    PulseCounter1 = 0;
    PulseCounter2 = 0;
}
unsigned long long L_GetTickCounter()
{
    return p_TickCounter;
}
void ParseRMC(void)
{
    uint8_t CommaCount = 0;
    uint8_t LoopCounter = 0;
    bool Skip = false;
    fixQuality = 0;
    while(LoopCounter<appData_GPS.GPRMC_Index)
    {
        if(appData_GPS.GPRMC[LoopCounter] == ',')
        {    
            Skip = false; 
            CommaCount++;
            LoopCounter++;
            continue;
        }
        if(CommaCount>2 && fixQuality==0)
            break;
        switch(CommaCount)
        {
            case 1: if(Skip==true) break; Skip = true; utcTime.hours =StrToInt(appData_GPS.GPRMC,LoopCounter,LoopCounter+1);
                    utcTime.minutes =StrToInt(appData_GPS.GPRMC,LoopCounter+2,LoopCounter+3);
                    utcTime.seconds =StrToInt(appData_GPS.GPRMC,LoopCounter+4,LoopCounter+5); break;  //get time
            case 2:if(appData_GPS.GPRMC[LoopCounter]=='A')fixQuality = 1; else fixQuality=0; break; //fix quality
            case 3: if(Skip==true) break; Skip = true; latitude.degrees = StrToInt(appData_GPS.GPRMC,LoopCounter,LoopCounter+1);
                    latitude.minutes = StrToInt(appData_GPS.GPRMC,LoopCounter+2,LoopCounter+3);
                    latitude.milliMinutes = StrToInt(appData_GPS.GPRMC,LoopCounter+5,LoopCounter+8);break;  //latitude
            case 4: latitude.northSouth = appData_GPS.GPRMC[LoopCounter]; break; //latitude sign
            case 5: if(Skip==true) break; Skip = true; longitude.degrees = StrToInt(appData_GPS.GPRMC,LoopCounter,LoopCounter+2);
                    longitude.minutes = StrToInt(appData_GPS.GPRMC,LoopCounter+3,LoopCounter+4);
                    longitude.milliMinutes = StrToInt(appData_GPS.GPRMC,LoopCounter+6,LoopCounter+9);break; //longitude
            case 6: longitude.eastWest = appData_GPS.GPRMC[LoopCounter]; break; // longitude sign
            case 7: if(Skip==true) break; Skip = true;gpsSpeed.knots =GetIntBeforeDec(appData_GPS.GPRMC,LoopCounter);gpsSpeed.deciKnots =GetIntAfterDec(appData_GPS.GPRMC,LoopCounter);   break; //speed
            case 8: if(Skip==true) break; Skip = true; gpsHeading.degrees =GetIntBeforeDec(appData_GPS.GPRMC,LoopCounter);gpsHeading.deciDegrees =GetIntAfterDec(appData_GPS.GPRMC,LoopCounter);  break; //angle
            case 9: if(Skip==true) break; Skip = true; gpsDate.day =StrToInt(appData_GPS.GPRMC,LoopCounter,LoopCounter+1);
                    gpsDate.month = StrToInt(appData_GPS.GPRMC,LoopCounter+2,LoopCounter+3); 
                    gpsDate.year = StrToInt(appData_GPS.GPRMC,LoopCounter+4,LoopCounter+5); 
                    FormatDate(false);
                    break; //date
        }
        LoopCounter++;
    }
}
uint16_t GetIntBeforeDec(uint8_t* Src,uint8_t Start)
{
   uint16_t ReturnValue = 0;
    uint8_t End = Start + 10;
    while(Start<End)
    {
        if(Src[Start]=='.'||Src[Start]==',')
            break;
        ReturnValue = ReturnValue*10;
        ReturnValue += (Src[Start]-48);
        Start++;
    }
    if(Start==End)  //number is invalid
        return 0;
    return ReturnValue;
}
uint16_t GetIntAfterDec(uint8_t* Src,uint8_t Start)
{
    uint16_t ReturnValue = 0;
    uint8_t End = Start + 10;
    while(Start<End)
    {
        if(Src[Start]=='.')
            break;
        Start++;
    }
    if(Start==End)  //number is invalid
        return 0;
    Start++;
    while(Start<End)
    {
        if(Src[Start]==',')
            break;
        ReturnValue = ReturnValue*10;
        ReturnValue += (Src[Start]-48);
        Start++;
    }
    if(Start==End)  //number is invalid
        return 0;
    return ReturnValue;
}
uint16_t StrToInt(uint8_t* Src,uint8_t Start,uint8_t End)
{
    uint16_t ReturnValue = 0;
    while(Start<=End)
    {
        ReturnValue = ReturnValue*10;
        ReturnValue += (Src[Start]-48);
        Start++;
    }
    return ReturnValue;
}
void ParseGGA(void)
{
    uint8_t CommaCount = 0;
    uint8_t LoopCounter = 0;
    bool Skip = false;
	while(LoopCounter<appData_GPS.GPGGA_Index)
    {
        if(appData_GPS.GPGGA[LoopCounter] == ',')
        {    
            Skip = false; 
            CommaCount++;
            LoopCounter++;
            continue;
        }
        switch(CommaCount)
        {
            case 7: if(Skip==true) break; Skip = true; satellitesCount = GetIntBeforeDec(appData_GPS.GPGGA,LoopCounter); break; //number of satellites
            case 9: if(Skip==true) break; Skip = true;gpsAltitude.metres =GetIntBeforeDec(appData_GPS.GPGGA,LoopCounter);gpsAltitude.deciMetres =GetIntAfterDec(appData_GPS.GPGGA,LoopCounter);   break; //speed
            case 10: if(Skip==true) break; Skip = true; gpsAltitude.mark = appData_GPS.GPGGA[LoopCounter]; break;
        }
        LoopCounter++;
    }
}
void GetRMC_GGMAMessags(void)
{
    size_t LoopIndex=0;
    while(LoopIndex<appData_GPS.drvUsart2RxBuffer_Index)
    {
        if(appData_GPS.FoundRMC == true)
        {
            if(appData_GPS.GPRMC_Index>99)
            {
                appData_GPS.GPRMC_Index = 0;
                appData_GPS.FoundRMC = false;
                continue;
            }
            if(appData_GPS.drvUsart2RxBuffer[LoopIndex]=='$')
            {
                //extract information here and fill the structs
                ParseRMC();
                appData_GPS.GPRMC_Index = 0;
                appData_GPS.FoundRMC = false;
                break;
            }
            appData_GPS.GPRMC[appData_GPS.GPRMC_Index] = appData_GPS.drvUsart2RxBuffer[LoopIndex];
            appData_GPS.GPRMC_Index++;
        }
        else
            if(appData_GPS.drvUsart2RxBuffer[LoopIndex]=='R')
                if(appData_GPS.drvUsart2RxBuffer[LoopIndex+1]=='M')
                    if(appData_GPS.drvUsart2RxBuffer[LoopIndex+2]=='C')
                    {
                        appData_GPS.FoundRMC = true;
                        appData_GPS.GPRMC_Index = 0;
                        continue;
                    }
        LoopIndex++ ;
    }
    
    LoopIndex=0;
    
    while(LoopIndex<appData_GPS.drvUsart2RxBuffer_Index)
    {
        if(appData_GPS.FoundGGA == true)
        {
            if(appData_GPS.GPGGA_Index>99)
            {
                appData_GPS.GPGGA_Index = 0;
                appData_GPS.FoundGGA = false;
                continue;
            }
            if(appData_GPS.drvUsart2RxBuffer[LoopIndex]=='$')
            {
                //extract information here and fill the structs
                ParseGGA();
                appData_GPS.GPGGA_Index = 0;
                appData_GPS.FoundGGA = false;
                break;
            }
            appData_GPS.GPGGA[appData_GPS.GPGGA_Index] = appData_GPS.drvUsart2RxBuffer[LoopIndex];
            appData_GPS.GPGGA_Index++;
        }
        else
            if(appData_GPS.drvUsart2RxBuffer[LoopIndex]=='G')
                if(appData_GPS.drvUsart2RxBuffer[LoopIndex+1]=='G')
                    if(appData_GPS.drvUsart2RxBuffer[LoopIndex+2]=='A')
                    {
                        appData_GPS.FoundGGA = true;
                        appData_GPS.GPGGA_Index = 0;
                        continue;
                    }
        LoopIndex++ ;
    }
    //appData_GPS.drvUsart2RxBuffer_Index = 0;
    
}
void GPS_SendUART2USB(void)
{
    p_USBDebugMessageSize=0;
    while(p_USBDebugMessageSize<appData_GPS.drvUsart2RxBuffer_Index)
    {
        p_USBDebugMessage[p_USBDebugMessageSize] = appData_GPS.drvUsart2RxBuffer[p_USBDebugMessageSize];
        p_USBDebugMessageSize++ ;
    }
    p_DebugMessagesCount=1;
    appData_GPS.drvUsart2RxBuffer_Index = 0;
}
uint8_t IntToStr(uint32_t Value, uint8_t* Buffer)
 {
 	uint8_t i;
 	uint32_t Digit;
 	uint32_t Divisor;
 	bool Printed = false;
    uint8_t BufferLength=0;
 	if(Value!=0)
 	{
 		for(i = 0, Divisor = 1000000; i < 7u; i++)
 		{
 			Digit = Value/Divisor;
 			if(Digit || Printed)
 			{
 				*Buffer++ = '0' + Digit;
 				Value -= Digit*Divisor;
 				Printed = true;
                BufferLength++;
 			}
 			Divisor /= 10;
 		}
 	}
 	else
 	{
 		*Buffer++ = '0';
        BufferLength++;
 	}
 
 	*Buffer = '\0';
    return BufferLength;
 }
uint8_t ftoa2(float fValue,  uint8_t* Buffer)
{
    uint32_t IntValue = (uint32_t)fValue;
    uint32_t FractionPart = fValue *100;
    uint8_t BufferLength = 0;
    FractionPart = FractionPart - (IntValue*100);
    
    BufferLength = IntToStr(IntValue, &Buffer[0]);
    Buffer[BufferLength++] = '.';
    BufferLength += IntToStr(FractionPart, &Buffer[BufferLength]);
    return BufferLength;
}
void XE910_CheckPower()
{
    if(appData_XE910.XE910_State==XE910_POWER_ON || appData_XE910.XE910_State==XE910_WAIT_TILL_ON)
        return;
    if(XE910_PWRMON()!=0)
        return;
    
    /*XE910 Module unexpectedly shut down. Reset the state to power it back on again */
    appData_XE910.XE910_State = XE910_POWER_ON ;
    
}
bool XE910_CheckContext()
{
    int ii=0;
    for(ii=0;ii<appData_XE910.XE910_Buffer_Index -1 ;ii++)
    {
        if(appData_XE910.drvXE910UsartRxBuffer[ii]=='T' &&appData_XE910.drvXE910UsartRxBuffer[ii+1]==':')
        {

            if(appData_XE910.drvXE910UsartRxBuffer[ii+5]=='1') 
            {
                /*context is active*/
                XE910_SendUART2USB();
                appData_XE910.XE910_Buffer_Index=0;
                return true;
            }
            if(appData_XE910.drvXE910UsartRxBuffer[ii+5]=='0')
            {
                /*Context is not active*/
                XE910_SendUART2USB();
                //appData.XE910_Buffer_Index=0;
                break;
            }
        }
    }
    return false;
}
bool XE910_CheckForOK()
{
   int ii=0;
   int X=0;
   for(ii=0;ii<appData_XE910.XE910_Buffer_Index -1 ;ii++)
    {
        if(appData_XE910.drvXE910UsartRxBuffer[ii]=='O' && appData_XE910.drvXE910UsartRxBuffer[ii+1]=='K')
        {
                if(appData_XE910.XE910ResponseState==AT_CGSN_RESPONSE)
                {
                        if(appData_XE910.XE910_Buffer_Index<27)
                                continue;
                        //read the IMEI
                        X=0;
                        ii = ii-19;
                        for(X=0;X<15;X++)
                        {
                                appData_XE910.IMEI[X] = appData_XE910.drvXE910UsartRxBuffer[ii];
                                ii++;
                        }
                }
                if(appData_XE910.XE910ResponseState==AT_CCLK_RESPONSE)
                {
                    ParseDateFromTelit(&appData_XE910.drvXE910UsartRxBuffer[0]);
                    FormatDate(true);
                }
                XE910_SendUART2USB();
                appData_XE910.XE910_Buffer_Index=0;
                return true;
        }
    }
   return false;
}
bool XE910_CheckForCONNECT()
{
   int ii=0;
   for(ii=0;ii<(appData_XE910.XE910_Buffer_Index -6) ;ii++)
    {
        if(appData_XE910.drvXE910UsartRxBuffer[ii]=='C' && appData_XE910.drvXE910UsartRxBuffer[ii+1]=='O')
            if(appData_XE910.drvXE910UsartRxBuffer[ii+2]=='N' && appData_XE910.drvXE910UsartRxBuffer[ii+3]=='N')
                if(appData_XE910.drvXE910UsartRxBuffer[ii+4]=='E' && appData_XE910.drvXE910UsartRxBuffer[ii+5]=='C')
                return true;
    }
   return false;
}
void XE910_SendUART2USB()
{
    p_USBDebugMessageSize=0;
    while(p_USBDebugMessageSize<appData_XE910.XE910_Buffer_Index)
    {
        p_USBDebugMessage[p_USBDebugMessageSize] = appData_XE910.drvXE910UsartRxBuffer[p_USBDebugMessageSize];
        p_USBDebugMessageSize++ ;
    }
    p_USBDebugMessage[p_USBDebugMessageSize] = '\r';
    p_USBDebugMessageSize++ ;
    p_USBDebugMessage[p_USBDebugMessageSize] = '\n';
    p_USBDebugMessageSize++;

    p_DebugMessagesCount=1;
}
void XE910_ClearXE910RXBuffer()
{
    int ii=0;
    for(ii=0;ii<1024;ii++)
    {
        appData_XE910.drvXE910UsartRxBuffer[ii] = 0;
    }
    appData_XE910.XE910_Buffer_Index = 0;
    
}
size_t L_CopyToArray(uint8_t* Dest,const char* Src)
{
    unsigned int _DestIndex = 0;

    while(*Src)
    {
        Dest[_DestIndex] = *Src++;
        _DestIndex++ ;
    }
    return _DestIndex;
}
void L_CopyArray1ToArray2(uint8_t* Src,uint8_t* Dest,uint16_t Count)
{
    unsigned int _DestIndex = 0;

    while(_DestIndex<Count)
    {
        Dest[_DestIndex] = Src[_DestIndex];
        _DestIndex++ ;
    }
}
void FormatDate(bool FixRTCFormat)
{
    if(FixRTCFormat==false)
    {
        //2015-11-17T13:00:32+00:00
        uint8_t AIndex = 0;
        AIndex+= IntToStr(20,&gpsDate.FullDateTime[AIndex]);
        AIndex+=IntToStr(gpsDate.year,&gpsDate.FullDateTime[AIndex]);
        gpsDate.FullDateTime[AIndex++] = '-';
        if(gpsDate.month<10)
        {
            gpsDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(gpsDate.month,&gpsDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(gpsDate.month,&gpsDate.FullDateTime[AIndex]);
        gpsDate.FullDateTime[AIndex++] = '-';
        if(gpsDate.day<10)
        {
            gpsDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(gpsDate.day,&gpsDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(gpsDate.day,&gpsDate.FullDateTime[AIndex]);
        gpsDate.FullDateTime[AIndex++] = 'T';
        if(utcTime.hours<10)
        {
            gpsDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(utcTime.hours,&gpsDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(utcTime.hours,&gpsDate.FullDateTime[AIndex]);
        gpsDate.FullDateTime[AIndex++] = ':';
        if(utcTime.minutes<10)
        {
            gpsDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(utcTime.minutes,&gpsDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(utcTime.minutes,&gpsDate.FullDateTime[AIndex]);
        gpsDate.FullDateTime[AIndex++] = ':';
        if(utcTime.seconds<10)
        {
            gpsDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(utcTime.seconds,&gpsDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(utcTime.seconds,&gpsDate.FullDateTime[AIndex]);
        gpsDate.FullDateTime[AIndex++] = '+';
        gpsDate.FullDateTime[AIndex++] = '0';
        gpsDate.FullDateTime[AIndex++] = '0';
        gpsDate.FullDateTime[AIndex++] = ':';
        gpsDate.FullDateTime[AIndex++] = '0';
        gpsDate.FullDateTime[AIndex++] = '0';
    }
    else
    {
        uint8_t AIndex = 0;
        AIndex+= IntToStr(20,&rtcDate.FullDateTime[AIndex]);
        AIndex+=IntToStr(rtcDate.year,&rtcDate.FullDateTime[AIndex]);
        rtcDate.FullDateTime[AIndex++] = '-';
        if(rtcDate.month<10)
        {
            rtcDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(rtcDate.month,&rtcDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(rtcDate.month,&rtcDate.FullDateTime[AIndex]);
        rtcDate.FullDateTime[AIndex++] = '-';
        if(rtcDate.day<10)
        {
            rtcDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(rtcDate.day,&rtcDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(rtcDate.day,&rtcDate.FullDateTime[AIndex]);
        rtcDate.FullDateTime[AIndex++] = 'T';
        if(rtcTime.hours<10)
        {
            rtcDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(rtcTime.hours,&rtcDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(rtcTime.hours,&rtcDate.FullDateTime[AIndex]);
        rtcDate.FullDateTime[AIndex++] = ':';
        if(rtcTime.minutes<10)
        {
            rtcDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(rtcTime.minutes,&rtcDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(rtcTime.minutes,&rtcDate.FullDateTime[AIndex]);
        rtcDate.FullDateTime[AIndex++] = ':';
        if(rtcTime.seconds<10)
        {
            rtcDate.FullDateTime[AIndex++] = '0';
            AIndex+=IntToStr(rtcTime.seconds,&rtcDate.FullDateTime[AIndex]);
        }
        else
            AIndex+=IntToStr(rtcTime.seconds,&rtcDate.FullDateTime[AIndex]);
        rtcDate.FullDateTime[AIndex++] = '+';
        rtcDate.FullDateTime[AIndex++] = '0';
        rtcDate.FullDateTime[AIndex++] = '1';
        rtcDate.FullDateTime[AIndex++] = ':';
        rtcDate.FullDateTime[AIndex++] = '0';
        rtcDate.FullDateTime[AIndex++] = '0';
    }
    
}
void BuildJSONPost_Accel()
{
    appData_XE910.XE910_CumulocityBufferIndex =  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[0],"{\"source\": {\"id\": \"1744800\"},\"text\": \"Measurement\",\"time\": \"");
    L_CopyArray1ToArray2(&rtcDate.FullDateTime[0],&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],25);
    appData_XE910.XE910_CumulocityBufferIndex  += 25;
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"\",\"type\": \"MeasurementUpdate\",\"c8y_Inputs\":");
    
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"{\"Accel\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(accelData.VectorData,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    //Accel_X
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"X\":{\"value\":");
    if(accelData.x<0)
    {
        appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"-");
        appData_XE910.XE910_CumulocityBufferIndex+= IntToStr((-1*accelData.x),&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    }
    else
        appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(accelData.x,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    //Accel_Y
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"Y\":{\"value\":");
    if(accelData.y<0)
    {
        appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"-");
        appData_XE910.XE910_CumulocityBufferIndex+= IntToStr((-1*accelData.y),&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    }
    else
        appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(accelData.y,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    //Accel_Z
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"Z\":{\"value\":");
    if(accelData.z<0)
    {
        appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"-");
        appData_XE910.XE910_CumulocityBufferIndex+= IntToStr((-1*accelData.z),&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    }
    else
        appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(accelData.z,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    //Magnetometer
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"Magn\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(magData.VectorData,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"}}");
}
void BuildJSONPost()
{
    //send these parameters: D1, D2, D3, D4, A1, A2, C1,C2,F1,F2
    appData_XE910.XE910_CumulocityBufferIndex =  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[0],"{\"source\": {\"id\": \"1744800\"},\"text\": \"Measurement\",\"time\": \"");
    L_CopyArray1ToArray2(&rtcDate.FullDateTime[0],&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],25);
    appData_XE910.XE910_CumulocityBufferIndex  += 25;
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"\",\"type\": \"MeasurementUpdate\",\"c8y_Inputs\":");
    //Digital input 1
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"{\"D1\":{\"value\":");
    appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex++] = GetIOStatus(1)+48;
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    //Digital Input 2
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"D2\":{\"value\":");
    appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex++] = GetIOStatus(2)+48;
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    
    //Digital Input 3
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"D3\":{\"value\":");
    appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex++] = GetIOStatus(3)+48;
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    
    //Digital Input 4
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"D4\":{\"value\":");
    appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex++] = GetIOStatus(4)+48;
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"\"}");
    
    //Analog Input 1
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"A1\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= ftoa2(ADCCH1,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"mV\"}");
    
    //Analog Input 2
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"A2\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= ftoa2(ADCCH2,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"mV\"}");
    
    //Current Input 1
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"C1\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= ftoa2(CurrentmA1,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"mA\"}");
    
    //Current Input 2
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"C2\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= ftoa2(CurrentmA2,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"mA\"}");
    
    //Frequency Input 1
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"F1\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(FrequencyIn1,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"RPM\"}");
    
    //Frequency Input 2
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"F2\":{\"value\":");
    appData_XE910.XE910_CumulocityBufferIndex+= IntToStr(FrequencyIn2,&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex]);
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],",\"unit\":\"RPM\"}");
    
    appData_XE910.XE910_CumulocityBufferIndex +=  L_CopyToArray(&appData_XE910.XE910_CumulocityBuffer[appData_XE910.XE910_CumulocityBufferIndex],"}}");
}
void ParseDateFromTelit(uint8_t* RXDate)
{
    uint8_t LoopCounter = 0;
    bool SearchDQ = false;
    for(LoopCounter=0;LoopCounter<70;LoopCounter++)
    {
        if(SearchDQ==false)
        {
            if( RXDate[LoopCounter]=='C')
            {
                if(RXDate[LoopCounter+1]=='L')
                    if(RXDate[LoopCounter+2]=='K')
                        if(RXDate[LoopCounter+3]==':')
                            SearchDQ = true;
            }
        }
        else
        {
            if(RXDate[LoopCounter]=='"')
            {
                LoopCounter++;
                rtcDate.year = StrToInt(RXDate,LoopCounter,LoopCounter+1);
                LoopCounter+=3;
                if(rtcDate.year<14 || rtcDate.year>30)
                {
                    //something is wrong
                    return;
                }
                rtcDate.month = StrToInt(RXDate,LoopCounter,LoopCounter+1);
                LoopCounter+=3;
                rtcDate.day = StrToInt(RXDate,LoopCounter,LoopCounter+1);
                LoopCounter+=3;
                rtcTime.hours = StrToInt(RXDate,LoopCounter,LoopCounter+1);
                LoopCounter+=3;
                rtcTime.minutes = StrToInt(RXDate,LoopCounter,LoopCounter+1);
                LoopCounter+=3;
                rtcTime.seconds = StrToInt(RXDate,LoopCounter,LoopCounter+1);
                appData_RTCC.state = RTCC_STATE_START; //start the RTCC state machine
                break;
            }
        }
    }
}
uint8_t BCD2Dec(uint8_t BCDValue)
{
    uint8_t DecValue = 0;
    DecValue = BCDValue >>4;
    DecValue = DecValue *10;
    BCDValue = BCDValue & 0b00001111;
    DecValue = DecValue + BCDValue;
    return DecValue;
}
uint8_t DEC2BCD(uint8_t DecValue)
{
    uint8_t BCDValue = 0;
    uint8_t tempVal;
    BCDValue = DecValue / 10;
    tempVal  = BCDValue *10;
    BCDValue = BCDValue <<4;
    BCDValue = BCDValue & 0b11110000;
    DecValue = DecValue - tempVal;
    BCDValue = BCDValue + DecValue;
    return BCDValue;
}
void ADC_ClearSPIRXBuffer()
{
    int ii=0;
    for(ii=0;ii<1024;ii++)
    {
        appData_MCP3910.drvSPI1RXbuffer[ii] = 0;
    }
}
/*******************************************************************************
 End of File
 */

