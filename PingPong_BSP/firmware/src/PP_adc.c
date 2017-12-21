/* 
 * File:   framework_adc12.h
 * Author: mgrundler
 *
 * Created on 28. Januar 2014, 19:41
 */
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "../framework/peripheral/peripheral.h"
#ifndef FRAMEWORK_ADC12_H
#define	FRAMEWORK_ADC12_H

#ifdef	__cplusplus
extern "C" {
#endif

// TODO: Take care of AN45-AN49! These are not "real" Channels! Their values are stored inside AD1DATA0 - AD1DATA4, and even the Ready-Bits of AN0-AN4 are used!!!

// <editor-fold defaultstate="collapsed" desc="ADC Defines">

//----------------- BEGIN ADC DEFINES ----------------------------
#define ADC_TAD 4000000 // Desired TAD clock Frequency (Use only interger value) (1MHz - 16MHz)
#define ADC_TAD_Sample_Time 8 // For shared S/H_5 chan, how many TAD clks to sample (SAMC: 1-256)
#define ADC_NUM_ANX_PINS 2 // How many shared channels will be used
#define ADC_Average_Bits 5 // will average (2^ADC_Average_Bits) samples
#define ADC_PINS_TO_SAMPLE {38,39} // Which ANx inputs will be sampled during the run
#define ADC_Slope 1155.0
#define ADC_Intercept 518.0
//----------------- END ADC DEFINES ------------------------------
#define UINT16 uint16_t
#define UINT32 uint32_t
#define UINT64 uint64_t
typedef enum
{
    MULTIPORT_CH1, // red
    MULTIPORT_CH2, // green
    MULTIPORT_CH3, // blue
    MULTIPORT_CH4, // white || amber
    FRONT_CH1,
    THERMISTOR,
    FAN_SPEED,
    FREQUENCY,
    MULTIPORT_CH1_STROBE, // red
    MULTIPORT_CH2_STROBE, // green
    MULTIPORT_CH3_STROBE, // blue
    MULTIPORT_CH4_STROBE, // white || amber
    FRONT_CH1_STROBE,
} ADC_CHANNEL_NAMES;

typedef volatile struct
{
    UINT16 ADC_Raw_Data[ADC_NUM_ANX_PINS][1<<ADC_Average_Bits];
    UINT32 AverageSums[ADC_NUM_ANX_PINS];
    UINT32 AverageSumsCheck[ADC_NUM_ANX_PINS];
    UINT16 ADC_Data_Average[ADC_NUM_ANX_PINS];
    uint8_t AveragePointer;
} ADC_DATA;

//----------------- USER DEFINES ERROR CHECKING ------------------
#if (ADC_TAD > 16000000) || (ADC_TAD < 1000000) || (ADC_TAD_Sample_Time < 3)
 #error "Invalid ADC_TAD or ADC_TAD_Sample_Time"
#endif
#if ((ADC_TAD / ADC_TAD_Sample_Time / ADC_NUM_ANX_PINS) < 4000)
 #warning "Samples will take longer than the control cycle"
#endif
//----------------- END USER DEFINES ERROR CHECKING --------------

//----------------- BEGIN OTHER DEFINES --------------------------
typedef uint8_t ANx_Array[ADC_NUM_ANX_PINS];
ANx_Array ANx_Pins = ADC_PINS_TO_SAMPLE;
double ADC_Slope_Inv = 1.0 / ADC_Slope;
double VDD_Inv = 1.0 / 3.3;
ADC_DATA ADCData;
//----------------- END OTHER DEFINES ----------------------------

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Initalize ADC">
void initADC2()
{
    int32_t results[5] ; // storage for results
    // This structure is used to simplify testing of the ready status
    AN_READY anReady ;
    AN_SELECT anSelect ;
    // This structure is used to simplify specifying channels for scan
    // AN5, AN11, AN12, AN13 and AN14 are selected for scanning.
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1);
    PLIB_PORTS_ChangeNoticePullUpPerPortDisable( PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1 );
    PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1,PORTS_PIN_MODE_ANALOG);
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0);
    PLIB_PORTS_ChangeNoticePullUpPerPortDisable( PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0 );
    PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0,PORTS_PIN_MODE_ANALOG);
    

    anSelect.highWord = 0 ;
    anSelect.lowWord = 0 ;
    anSelect.AN39 = 1 ;
    anSelect.AN38 = 1 ;
    PLIB_ADCP_Configure ( ADCP_ID_1 ,
    ADCP_VREF_AVDD_AVSS , // AVDD and AVSS as reference
    false , // No VREF boost
    false , // Do not use fractional format
    false , // Do not stop in idle
    ADCP_CLK_SRC_SYSCLK , // SYSCLK is the clock source
    4 , // TAD = 1/SYSCLK * 2 * 4
    // or ADC Clock = SYSCLK / (2 * 4)
    0 , // Oversampling is not used.
    0 , // No early interrupt.
    3 ) ; // 3 + 1 = 4 TAD for Class 2 and 3
    // Sample Time.
    // Specify the inputs to include in the Channel Scan using the selections made when
    // initializing anSelect. Use the software trigger to initiate the scan.
    PLIB_ADCP_ChannelScanConfigure ( ADCP_ID_1 ,
    anSelect.lowWord , anSelect.highWord ,
    ADCP_SCAN_TRG_SRC_SOFTWARE ) ;
    PLIB_ADCP_SHModeSelect ( ADCP_ID_1 , ADCP_SH5 , ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR ) ;
    PLIB_ADCP_Enable ( ADCP_ID_1 ) ;

    PLIB_ADCP_GlobalSoftwareTrigger ( ADCP_ID_1 ) ;
    
}
void initADC()
{
    int i;
    uint64_t scanbits = 0;
    AN_SELECT anSelect ;
    // <editor-fold defaultstate="collapsed" desc="setup pins">

    // B4 (AN4) --> PIN 30 (AN4) on HEADER J11

    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1);
    PLIB_PORTS_ChangeNoticePullUpPerPortDisable( PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1 );
    PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_1,PORTS_PIN_MODE_ANALOG);
    PLIB_PORTS_PinDirectionInputSet(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0);
    PLIB_PORTS_ChangeNoticePullUpPerPortDisable( PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0 );
    PLIB_PORTS_PinModePerPortSelect(PORTS_ID_0, PORT_CHANNEL_H, PORTS_BIT_POS_0,PORTS_PIN_MODE_ANALOG);

    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="setup ADC">

    // Set up the channel scan
    for (i = 0; i < ADC_NUM_ANX_PINS; i++)
    {
        scanbits += (UINT64)(1 << ANx_Pins[i]);
    }
    scanbits = 0xC000000000;
    anSelect.highWord = 0 ;
    anSelect.lowWord = 0 ;
    anSelect.AN38=1;
    anSelect.AN39 = 1;
    // configure ADC
    PLIB_ADCP_Configure(ADCP_ID_1, ADCP_VREF_AVDD_AVSS, false, false, false, ADCP_CLK_SRC_FRC, 8, 0, 0, 3);
// PLIB_ADCP_Configure(ADCP_ID_1, ADCP_VREF_VREFP_AVSS, false, false, false, ADCP_CLK_SRC_SYSCLK, (SYS_CLK_SystemFrequencyGet() / ADC_TAD / 2), 32, 0, 32);
// PLIB_ADCP_Configure(ADCP_ID_1, ADCP_VREF_AVDD_VREFN, false, false, false, ADCP_CLK_SRC_SYSCLK, (SYS_CLK_SystemFrequencyGet() / ADC_TAD / 2), 32, 0, 32);
// PLIB_ADCP_Configure(ADCP_ID_1, ADCP_VREF_VREFP_VREFN, false, false, false, ADCP_CLK_SRC_SYSCLK, (SYS_CLK_SystemFrequencyGet() / ADC_TAD / 2), 32, 0, 32);

    AD1CAL1 = 0xF8894530; // Use software calibration values into AD1CALx
    AD1CAL2 = 0x01E4AF69;
    AD1CAL3 = 0x0FBBBBB8;
    AD1CAL4 = 0x000004AC;
    AD1CAL5 = 0x02000002;

    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH0, ADCP_SH_MODE_DIFFERENTIAL_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH1, ADCP_SH_MODE_DIFFERENTIAL_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH2, ADCP_SH_MODE_DIFFERENTIAL_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH3, ADCP_SH_MODE_DIFFERENTIAL_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH4, ADCP_SH_MODE_DIFFERENTIAL_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH5, ADCP_SH_MODE_DIFFERENTIAL_UNIPOLAR);

// PLIB_ADCP_ChannelScanConfigure(ADCP_ID_1, 0, 0, ADCP_SCAN_TRG_SRC_SOFTWARE);
    PLIB_ADCP_ChannelScanConfigure(ADCP_ID_1, anSelect.lowWord , anSelect.highWord, ADCP_SCAN_TRG_SRC_SOFTWARE);
    PLIB_ADCP_Enable ( ADCP_ID_1 ) ; // enable ADC module
    while(!PLIB_ADCP_ModuleIsReady ( ADCP_ID_1 )) ; // wait for ADC ready

    PLIB_ADCP_CalibrationStart(ADCP_ID_1); // calibrate ADC module
    while(!PLIB_ADCP_ModuleIsReady ( ADCP_ID_1 )) ; // wait for ADC ready

    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH0, ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH1, ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH2, ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH3, ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH4, ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR);
    PLIB_ADCP_SHModeSelect(ADCP_ID_1, ADCP_SH5, ADCP_SH_MODE_SINGLE_ENDED_UNIPOLAR);



    // triger an ADC sample
    PLIB_ADCP_GlobalSoftwareTrigger ( ADCP_ID_1 ) ;

    // </editor-fold>

}

bool ADCDataReady()
{
    int i;
    AN_READY anReady ;
    anReady = PLIB_ADCP_ResultReady ( ADCP_ID_1 );

    if(anReady.AN39==0)
        return false;
    else
        return true;
    for (i = 0; i < ADC_NUM_ANX_PINS; i++)
    {
        if ( (anReady.ret_val & (1 << ANx_Pins[i])) == 0 )
        {
            return false;
        }
    }

    return true;
}

int32_t calculateVoltage(uint8_t channel)
{
    if(channel==0)
        return PLIB_ADCP_ResultGet(ADCP_ID_1 , ADCP_AN38)-467;
    else
        return PLIB_ADCP_ResultGet(ADCP_ID_1 , ADCP_AN39)-467;
    return ((double)ADCData.ADC_Data_Average[channel]);// - ADC_Intercept) * ADC_Slope_Inv;
}

double calculatePercentADC(uint8_t channel)
{
    return calculateVoltage(channel) * VDD_Inv;
}

void ADCProcessValues()
{
    int i;

    for ( i = 0 ; i < ADC_NUM_ANX_PINS ; i++ )
    {
        ADCData.AverageSums[i] -= ADCData.ADC_Raw_Data[i][ADCData.AveragePointer];
        ADCData.ADC_Raw_Data[i][ADCData.AveragePointer] = PLIB_ADCP_ResultGet(ADCP_ID_1, ANx_Pins[i]);
        ADCData.AverageSums[i] += ADCData.ADC_Raw_Data[i][ADCData.AveragePointer];
        ADCData.ADC_Data_Average[i] = ADCData.AverageSums[i] >> ADC_Average_Bits;

    }

    // incriment pointer
    ADCData.AveragePointer = (ADCData.AveragePointer+1) & ((1<<ADC_Average_Bits)-1);

}

// </editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* FRAMEWORK_ADC12_H */

