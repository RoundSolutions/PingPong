/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "sqi.h"
#include "peripheral/sqi/plib_sqi.h"
#include "peripheral/osc/plib_osc.h"
// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
    // *****************************************************************************
    
 #define APP_READ_BUFFER_SIZE 512
/* Macro defines USB internal DMA Buffer criteria*/

#define APP_MAKE_BUFFER_DMA_READY  __attribute__((coherent)) __attribute__((aligned(4)))
    
// *************************Control Pins definitions****************************
// *****************************************************************************
#define MAX_NUM_OF_BYTES_IN_BUF 1024
#define	ADC_CS          PORTS_ID_0,PORT_CHANNEL_D, PORTS_BIT_POS_9
#define	ADC_Select      PORTS_ID_0,PORT_CHANNEL_K, PORTS_BIT_POS_5

#define	nInt_Ethernet	PORTS_ID_0,PORT_CHANNEL_F, PORTS_BIT_POS_2
#define	nRST_Ethernet	PORTS_ID_0,PORT_CHANNEL_F, PORTS_BIT_POS_8
#define	CAN_STDBY       PORTS_ID_0,PORT_CHANNEL_K, PORTS_BIT_POS_7

#define	SQI_PWR         PORTS_ID_0,PORT_CHANNEL_J, PORTS_BIT_POS_6
#define	SIM_SELECTTION	PORTS_ID_0,PORT_CHANNEL_J, PORTS_BIT_POS_7
#define	BLUE_LED        PORTS_ID_0,PORT_CHANNEL_E, PORTS_BIT_POS_3
#define	GREEN_LED       PORTS_ID_0,PORT_CHANNEL_E, PORTS_BIT_POS_4
#define	ADC_DR          PORTS_ID_0,PORT_CHANNEL_K, PORTS_BIT_POS_6
#define	ACC_INT1        PORTS_ID_0,PORT_CHANNEL_A, PORTS_BIT_POS_4
#define	ACC_INT2        PORTS_ID_0,PORT_CHANNEL_H, PORTS_BIT_POS_11

#define	DI1G            PORTS_ID_0,PORT_CHANNEL_B, PORTS_BIT_POS_6
#define	DI1P            PORTS_ID_0,PORT_CHANNEL_B, PORTS_BIT_POS_7
#define	DI2G            PORTS_ID_0,PORT_CHANNEL_A, PORTS_BIT_POS_9
#define	DI2P            PORTS_ID_0,PORT_CHANNEL_A, PORTS_BIT_POS_10
#define	DI3G            PORTS_ID_0,PORT_CHANNEL_H, PORTS_BIT_POS_15
#define	DI3P            PORTS_ID_0,PORT_CHANNEL_K, PORTS_BIT_POS_1
#define	DI4G            PORTS_ID_0,PORT_CHANNEL_K, PORTS_BIT_POS_2
#define	DI4P            PORTS_ID_0,PORT_CHANNEL_K, PORTS_BIT_POS_3
#define	DO1             PORTS_ID_0,PORT_CHANNEL_H, PORTS_BIT_POS_7
#define	DO2             PORTS_ID_0,PORT_CHANNEL_D, PORTS_BIT_POS_14
#define	DO3             PORTS_ID_0,PORT_CHANNEL_D, PORTS_BIT_POS_15
#define	DO4             PORTS_ID_0,PORT_CHANNEL_B, PORTS_BIT_POS_12
#define	GPS_PPS         PORTS_ID_0,PORT_CHANNEL_D, PORTS_BIT_POS_13
#define	XE910_DCD       PORTS_ID_0,PORT_CHANNEL_G, PORTS_BIT_POS_0
#define	XE910__PWRMON	PORTS_ID_0,PORT_CHANNEL_J, PORTS_BIT_POS_4
#define	XE910_Switch	PORTS_ID_0,PORT_CHANNEL_J, PORTS_BIT_POS_2
#define	XE910_ON_OFF	PORTS_ID_0,PORT_CHANNEL_J, PORTS_BIT_POS_3
#define	XE910_OE        PORTS_ID_0,PORT_CHANNEL_G, PORTS_BIT_POS_15
#define	XE910_SPI_MRDY	PORTS_ID_0,PORT_CHANNEL_B, PORTS_BIT_POS_8
#define	XE910_SPI_SRDY  PORTS_ID_0,PORT_CHANNEL_B, PORTS_BIT_POS_11
#define	XE910_SHTDN     PORTS_ID_0,PORT_CHANNEL_G, PORTS_BIT_POS_1
#define	Max_1_WIRE      PORTS_ID_0,PORT_CHANNEL_H, PORTS_BIT_POS_2


#define BLUE_LED_ON()           PLIB_PORTS_PinClear(BLUE_LED)
#define BLUE_LED_OFF()          PLIB_PORTS_PinSet(BLUE_LED)
#define GREEN_LED_ON()          PLIB_PORTS_PinClear(GREEN_LED)
#define GREEN_LED_OFF()         PLIB_PORTS_PinSet(GREEN_LED)
#define BLUE_LED_Toggle()       PLIB_PORTS_PinToggle(BLUE_LED)
#define GREEN_LED_Toggle()      PLIB_PORTS_PinToggle(GREEN_LED)
#define XE910_PWRMON()          PLIB_PORTS_PinGet(XE910__PWRMON)
#define XE910_IsNotConnected()     PLIB_PORTS_PinGet(XE910_DCD)
#define GetPin                  PLIB_PORTS_PinGet
#define MCP3910_Resistor        49.f
#define MCP3910_VREF            1.2f
//Acceleromer definitions
#define FXOS8700CQ_SLAVE_ADDR 0x3C // with pins SA0=0, SA1=0
#define FXOS8700CQ_STATUS 0x00
#define FXOS8700CQ_WHOAMI 0x0D
#define FXOS8700CQ_XYZ_DATA_CFG 0x0E
#define FXOS8700CQ_CTRL_REG1 0x2A
#define FXOS8700CQ_M_CTRL_REG1 0x5B
#define FXOS8700CQ_M_CTRL_REG2 0x5C
#define FXOS8700CQ_WHOAMI_VAL 0xC7
#define FXOS8711CQFXOS8701CQFXOS8700CQ_READ_LEN 13// status plus 6 channels = 13 bytes
typedef struct
{
int16_t x;
int16_t y;
int16_t z;
uint32_t VectorData;
} ACCELDATA;
typedef struct
{
int16_t x;
int16_t y;
int16_t z;
uint32_t VectorData;
} MAGDATA;
enum
{
    LOGIC_HIGH=0,
    LOGIC_LOW=1,
    FLOATING=2

};
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
/* Application's state machine's initial state. */
    APP_STATE_INIT=0,
    APP_STATE_CHECK_DRVR_STATE,
    APP_STATE_GetGPSData,
    /* Application waits for device configuration*/
    APP_STATE_WAIT_FOR_CONFIGURATION,

    /* The application checks if a switch was pressed */
    APP_STATE_CHECK_SWITCH_PRESSED,

    /* Wait for a character receive */
    APP_STATE_SCHEDULE_READ,

    /* A character is received from host */
    APP_STATE_WAIT_FOR_READ_COMPLETE,

    /* Wait for the TX to get completed */
    APP_STATE_SCHEDULE_WRITE,

    /* Wait for the write to complete */
    APP_STATE_WAIT_FOR_WRITE_COMPLETE,

    //APP States for XE910
    XE910_POWER_ON,
    XE910_WAIT_TILL_ON,
    XE910_SETUP_USART,
    XE910_TIMEOUT_WAIT,
    XE910_SET_GPIO,
    XE910_SET_CMEE,
    XE910_GET_CPIN,
    XE910_GET_IMEI,
    XE910_SET_FLOWCONTROL,
    XE910_SET_SELINT,
    XE910_SET_SCFG,
    XE910_SET_APN,
    XE910_CHECK_CONTEXT,
    XE910_ACTIVATE_CONTEXT,
    XE910_OPEN_SOCKET,
    XE910_SEND_DATA,
    XE910_DEACTIVATE_CONTEXT,
    XE910_Delay,
    APP_STATE_NOTHING,
    /*AT Commands wait states*/
    AT_RESPONSE,
    AT_GPIO_RESPONSE,
    AT_CMEE_RESPONSE,
    AT_CPIN_RESPONSE,
    AT_CGSN_RESPONSE,
    AT_ANDC1_RESPONSE,
    AT_SELINT_RESPONSE,
    AT_SCFG_RESPONSE,
    AT_CGDCONT_RESPONSE,
    AT_SGACT_Q_RESPONSE,
    AT_SGACT_RESPONSE,
    AT_MONI_RESPONSE,
    AT_SD_RESPONSE,
    AT_CCLK_RESPONSE,
            
    AT_SEND,
    AT_GPIO_SEND,
    AT_CMEE_SEND,
    AT_CPIN_SEND,
    AT_CGSN_SEND,
    AT_ANDC1_SEND,
    AT_SELINT_SEND,
    AT_SCFG_SEND,
    AT_CGDCONT_SEND,
    AT_SGACT_Q_SEND,
    AT_SGACT_SEND,
    AT_MONI_SEND,
    AT_SD_SEND,    
    AT_CCLK_SEND,
    
    ACCEL_RD_WHOAMI,
    ACCE_WAIT_FOR_COMPLETE,
    ACCEL_STDBY,
    ACCEL_CONFIGMAG_REG1,
    ACCEL_CONFIGMAG_REG2,
    ACCEL_XYZ_CONFIG,
    ACCEL_CONFIG_REG1,
    ACCEL_ReadData,

    RTCC_STATE_START,
    RTCC_WAIT_CLOCk_OFF,
    RTCC_SET_DATETIME,
    RTCC_READ_DATETIME,
    RTCC_WAIT_FOR_SOURCE,
    //SPI states
    APP_STATE_CONFIG_1,
    APP_STATE_WAIT_CONFIG1,
    APP_STATE_READ_CH0,
    APP_STATE_WAIT_READ_COMPLETE,
    APP_STATE_READ_DONE,
    APP_STATE_RESET,
    APP_STATE_RESUME,
    //SQI states
    APP_STATE_FLASH_ID_READ,

    /* In this state, the application writes to flash. */
    APP_STATE_WRITE_FLASH,

    /* In this state, the application reads from flash in PIO mode */
    APP_STATE_READ_FLASH_PIO_MODE,

    /* In this state, the application reads from flash in XIP mode */
    APP_STATE_READ_FLASH_XIP_MODE,

    /* In this state, the application reads from flash in DMA mode */
    APP_STATE_READ_FLASH_DMA_MODE,

    /* Blinks LED indicating completion */
    APP_STATE_DONE,
    /* Application Error state*/
    APP_STATE_ERROR
            
            

} APP_STATES;
typedef enum{

    WR_SEND_DATA_TO_EEPROM = 0,
    WR_SEND_DATA_TO_I2C_SLAVE,
    WR_WAIT_FOR_EEPROM_WRITE_COMPLETE,
    WR_WAIT_FOR_STATUS_REPLY,
    WR_COMPLETED

}APP_EEPROM_WR_STATES;

typedef enum{

    RD_SEND_STATUS_CODE_CMD = 0,
    RD_WAIT_FOR_STATUS_CMD_REPLY,
    RD_GET_STATUS_DATA,
    RD_BUSY_STATUS_CHECK,
    RD_DATA_FROM_I2C_SLAVE,
    RD_WAIT_FOR_I2C_COMPLETE,
    RD_COMPLETE

}APP_EEPROM_RD_STATES;

#define MAX_READINGS 2

typedef struct
{
	void * SPI1TxBufferRef;
	void * SPI1RxBufferRef;

}context;


typedef struct{
    APP_STATES state;
    unsigned long long p_SPITimeout;
    DRV_HANDLE  drvSPI1Handle;
    DRV_SPI_BUFFER_HANDLE   drvSPI1TxBufHandle;
    DRV_SPI_BUFFER_HANDLE   drvSPI1RxBufHandle;
    uint8_t drvSPI1TXbuffer[MAX_NUM_OF_BYTES_IN_BUF];
    uint8_t drvSPI1RXbuffer[MAX_NUM_OF_BYTES_IN_BUF];
    size_t SPI_Buffer_Index;
    context gBufferContext;
    uint32_t adcSamples;
    float adcMeasurement;
    
} APPDATA_MCP3910;
// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Device layer handle returned by device layer open function */
    USB_DEVICE_HANDLE deviceHandle;

    /* Application's current state*/
    APP_STATES state;

    /* Set Line Coding Data */
    USB_CDC_LINE_CODING setLineCodingData;

    /* Device configured state */
    bool isConfigured;

    /* Get Line Coding Data */
    USB_CDC_LINE_CODING getLineCodingData;

    /* Control Line State */
    USB_CDC_CONTROL_LINE_STATE controlLineStateData;

    /* Read transfer handle */
    USB_DEVICE_CDC_TRANSFER_HANDLE readTransferHandle;

    /* Write transfer handle */
    USB_DEVICE_CDC_TRANSFER_HANDLE writeTransferHandle;

    /* True if a character was read */
    bool isReadComplete;

    /* True if a character was written*/
    bool isWriteComplete;

    /* True is switch was pressed */
    bool isSwitchPressed;

    /* True if the switch press needs to be ignored*/
    bool ignoreSwitchPress;

    /* Flag determines SOF event occurrence */
    bool sofEventHasOccurred;

    /* Break data */
    uint16_t breakData;

    /* Switch debounce timer */
    unsigned int switchDebounceTimer;

    unsigned int debounceCount;

    /* Application CDC read buffer */
    uint8_t * readBuffer;
    /* TODO: Define any additional data used by the application. */


} APP_DATA_USB;

typedef struct
{
    DRV_HANDLE  drvUsart2Handle;
    APP_STATES  state;
    uint8_t  drvUsart2TxBuffer[1024];
    uint8_t  drvUsart2RxBuffer[1024];
    uint8_t GPRMC[100];
    uint8_t GPRMC_Index;
    uint8_t GPGGA_Index;
    uint8_t GPGGA[100];
    bool FoundRMC;
    bool FoundGGA;
    size_t   drvUsart2RxBuffer_Index;
    DRV_USART_BUFFER_HANDLE   drvUsart2TxBufHandle;
    DRV_USART_BUFFER_HANDLE   drvUsart2RxBufHandle;
    
}APP_DATA_GPS;


typedef struct
{
    APP_STATES  state;
    
}APPData_Flash;

typedef struct
{
    APP_STATES  state;
    
}APPData_RTCC;
typedef struct
{
    APP_STATES  state;
    SYS_MODULE_OBJ          drvI2CObject;
    DRV_HANDLE              drvI2CHandle;
    DRV_I2C_BUFFER_HANDLE   drvI2CWRBUFHandle;
    DRV_I2C_BUFFER_HANDLE   drvI2CRDBUFHandle;
    I2C_DATA_TYPE           drvI2CTXbuffer[MAX_NUM_OF_BYTES_IN_BUF];
    I2C_DATA_TYPE           drvI2CRXbuffer[MAX_NUM_OF_BYTES_IN_BUF];
}APP_DATA_ACCEL;
typedef struct
{
    APP_STATES  state;
}APP_DATA_IC;
typedef struct
{
    /* The application's current state */
    APP_STATES state;
    APP_STATES XE910_State;
    char XE910_RXByte;
    char XE910_RXBuffer[1024];
    char XE910_CumulocityBuffer[768];
    int XE910_RXBufferIndex;
    uint16_t XE910_CumulocityBufferIndex;

        /* Write buffer handle */
    DRV_USART_BUFFER_HANDLE   drvUsart2TxBufHandle;
    DRV_USART_BUFFER_HANDLE   drvUsart2RxBufHandle;

    DRV_USART_BUFFER_HANDLE   drvXE910UsartTxBufHandle;
    DRV_USART_BUFFER_HANDLE   drvXE910UsartRxBufHandle;
    DRV_HANDLE  drvXE910UsartHandle;
    APP_STATES  XE910UsartState;
    APP_STATES  XE910ResponseState;
    uint8_t  drvXE910UsartTxBuffer[MAX_NUM_OF_BYTES_IN_BUF];
    uint8_t  drvXE910UsartRxBuffer[MAX_NUM_OF_BYTES_IN_BUF];
    size_t   XE910_Buffer_Index;
    uint8_t IMEI[15];
    
    bool TxIntervalWait;
}APP_DATA_XE910;

typedef struct  {
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
} UtcTime;

typedef struct  {
        uint8_t degrees;
        uint8_t minutes;
        uint16_t milliMinutes;
        char northSouth;
} Latitude;

typedef struct  {
        uint8_t degrees;
        uint8_t minutes;
        uint16_t milliMinutes;
        char eastWest;
} Longitude;

typedef struct  {
    uint8_t day;
    uint8_t month;
    uint8_t year;
    char FullDateTime[26];
} GPSDate;

typedef struct {
    uint16_t knots;
    uint16_t deciKnots;
} GPSSpeed;
typedef struct {
    uint16_t degrees;
    uint8_t deciDegrees;
} GPSHeading;
typedef struct {
    uint16_t metres;
    uint16_t deciMetres;
    char mark;
} GPSAltitude;
UtcTime utcTime;
UtcTime rtcTime;
Latitude latitude;
Longitude longitude;
GPSDate gpsDate;
GPSDate rtcDate;
uint8_t fixQuality;
uint8_t satellitesCount;
GPSSpeed gpsSpeed;
GPSHeading gpsHeading;
GPSAltitude gpsAltitude;
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
  Function:
    void APP_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void APP_Tasks_USB( void );
void App_Tasks_General(void);
void App_Tasks_GPS(void);
void App_Task_XE910(void);
void App_Tasks_ACCEL( void );
void App_Task_Flash( void );
void App_Task_1Wire( void );
void App_Tasks_RTC ( void );
void APP_BufferEventHandlerUsart2(DRV_USART_BUFFER_EVENT buffEvent,DRV_USART_BUFFER_HANDLE hBufferEvent,uintptr_t context );
void APP_BufferEventHandlerXE910Usart(DRV_USART_BUFFER_EVENT buffEvent,DRV_USART_BUFFER_HANDLE hBufferEvent,uintptr_t context );
void APP_I2C_MasterOpEndCallback (DRV_I2C_BUFFER_EVENT event,DRV_I2C_BUFFER_HANDLE bufferHandle,uintptr_t context);
void APP_BufferEventHandlerSPI1(DRV_SPI_BUFFER_EVENT buffEvent,DRV_SPI_BUFFER_HANDLE hBufferEvent,void* context );
void APP_Tasks_MCP3910(void);
/********Application Variables******/
size_t RXDatalength;
DRV_HANDLE Timer_handle;
unsigned long long p_TickCounter;
unsigned long long p_GeneralCounter;
unsigned long long p_GPSTimeout;
unsigned long long p_XE910Timeout;
unsigned long long p_AccelTimeout;
unsigned long long p_1WireTimeOut;
unsigned long long p_RTCCTimeOut;
bool AccelDataReady;
bool CheckAgain;
int32_t p_ADValue;
int32_t p_ADValue2;
float ADCCH1;
float ADCCH2;
float CurrentmA1;
float CurrentmA2;
APP_EEPROM_RD_STATES readstate;
APP_EEPROM_WR_STATES writestate;
uint8_t AccelAddress;
uint32_t PulseCounter1;
uint32_t PulseCounter2;
uint32_t FrequencyIn1;
uint32_t FrequencyIn2;
uint8_t MySeconds;
bool FoundSec;
/********Custom Functions**********/
uint8_t GetIOStatus(uint8_t PortNumber);
void SendUSBDebugMessage(const char* Src);
void Timer_CounterCallback(uintptr_t context, uint32_t alarmCount);
unsigned long long L_GetTickCounter(void);
void GPS_SendUART2USB(void);
void GetRMC_GGMAMessags(void);
void ParseRMC(void);
void ParseGGA(void);
uint16_t StrToInt(uint8_t* Src,uint8_t Start,uint8_t End);
uint16_t GetIntBeforeDec(uint8_t* Src,uint8_t Start);
uint16_t GetIntAfterDec(uint8_t* Src,uint8_t Start);
uint8_t IntToStr(uint32_t Value, uint8_t* Buffer);
uint8_t ftoa2(float fValue,  uint8_t* Buffer);
void XE910_CheckPower(void);
bool XE910_CheckForOK(void);
bool XE910_CheckContext(void);
void XE910_SendUART2USB(void);
void XE910_ClearXE910RXBuffer(void);
size_t L_CopyToArray(uint8_t* Dest,const char* Src);
void FormatDate(bool FixRTCFormat);
void L_CopyArray1ToArray2(uint8_t* Src,uint8_t* Dest,uint16_t Count);
bool XE910_CheckForCONNECT(void);
void BuildJSONPost(void);
void BuildJSONPost_Accel(void);
void App_Tasks_IC(void);
void ParseDateFromTelit(uint8_t* RXDate);
uint8_t BCD2Dec(uint8_t BCDValue);
uint8_t DEC2BCD(uint8_t DecValue);
void ADC_ClearSPIRXBuffer(void);
#endif /* _APP_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

