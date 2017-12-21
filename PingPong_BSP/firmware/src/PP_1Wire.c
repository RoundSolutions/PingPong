#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "../framework/peripheral/peripheral.h"
#include "peripheral/ports/plib_ports.h"
#include "peripheral/int/plib_int.h"
#ifndef FRAMEWORK_1WIRE_H
#define	FRAMEWORK_1WIRE_H
#define	Max_1_WIRE               PORTS_ID_0,PORT_CHANNEL_H, PORTS_BIT_POS_2
#define outp_MAX0()              PLIB_PORTS_PinDirectionOutputSet( Max_1_WIRE);PLIB_PORTS_PinClear(Max_1_WIRE)
#define outp_MAX1()              PLIB_PORTS_PinDirectionInputSet(Max_1_WIRE)
#define inp_MAX()                PLIB_PORTS_PinGet(Max_1_WIRE)    
//core time ticks 100M     
#ifdef	__cplusplus
extern "C" {
#endif
 int A,B,C,D,E,F,G,H,I,J;   
// <editor-fold defaultstate="collapsed" desc="Functions">
 static uint32_t APP_ReadCoreTimer()
{
    volatile uint32_t timer;

    // get the count reg
    asm volatile("mfc0   %0, $9" : "=r"(timer));

    return(timer);
}
 void ShortDelay(uint32_t DelayCount)                   // Delay Time (CoreTimer Ticks)
{
  uint32_t StartTime;                    // Start Time
  StartTime = APP_ReadCoreTimer();         // Get CoreTimer value for StartTime
  while ( (uint32_t)(APP_ReadCoreTimer() - StartTime) < DelayCount ) {};
}
 void tickDelay(int mydelay)
{
     //create a mydelay x 0.25 us delay here
     ShortDelay(mydelay*25);//0.25us delay here  
}
void Set1WireSpeed(int standard)
{
        // Adjust tick values depending on speed
        if (standard)
        {
                // Standard Speed
                A = 6 * 4;
                B = 64 * 4;
                C = 60 * 4;
                D = 10 * 4;
                E = 9 * 4;
                F = 55 * 4;
                G = 0;
                H = 480 * 4;
                I = 70 * 4;
                J = 410 * 4;
        }
        else
        {
                // Overdrive Speed
                A = 1.5 * 4;
                B = 7.5 * 4;
                C = 7.5 * 4;
                D = 2.5 * 4;
                E = 0.75 * 4;
                F = 7 * 4;
                G = 2.5 * 4;
                H = 70 * 4;
                I = 8.5 * 4;
                J = 40 * 4;
        }
}
// Generate a 1-Wire reset, return 1 if no presence detect was found,
// return 0 otherwise.
// (NOTE: Does not handle alarm presence from DS2404/DS1994)
//
int OWTouchReset(void)
{
        int result;

        tickDelay(G);
        outp_MAX0() ; // Drives DQ low
        tickDelay(H);
        outp_MAX1() ; // Releases the bus
        tickDelay(I);
        result = inp_MAX() ^ 0x01; // Sample for presence pulse from slave
        tickDelay(J); // Complete the reset sequence recovery
        return result; // Return sample presence pulse result
}
//-----------------------------------------------------------------------------
// Send a 1-Wire write bit. Provide 10us recovery time.
//
void OWWriteBit(int bit)
{
        if (bit)
        {
                // Write '1' bit
                outp_MAX0() ; // Drives DQ low
                tickDelay(A);
                outp_MAX1() ; // Releases the bus
                tickDelay(B); // Complete the time slot and 10us recovery
        }
        else
        {
                // Write '0' bit
                outp_MAX0() ; // Drives DQ low
                tickDelay(C);
                outp_MAX1() ; // Releases the bus
                tickDelay(D);
        }
}

//-----------------------------------------------------------------------------
// Read a bit from the 1-Wire bus and return it. Provide 10us recovery time.
//
int OWReadBit(void)
{
        int result;

        outp_MAX0() ; // Drives DQ low
        tickDelay(A);
        outp_MAX1() ; // Releases the bus
        tickDelay(E);
        result = inp_MAX(); // Sample the bit value from the slave
        tickDelay(F); // Complete the time slot and 10us recovery

        return result;
}
//-----------------------------------------------------------------------------
// Write 1-Wire data byte
//
void OWWriteByte(int data)
{
        int loop;

        // Loop to write each bit in the byte, LS-bit first
        for (loop = 0; loop < 8; loop++)
        {
                OWWriteBit(data & 0x01);

                // shift the data byte for the next bit
                data >>= 1;
        }
}

//-----------------------------------------------------------------------------
// Read 1-Wire data byte and return it
//
int OWReadByte(void)
{
        int loop, result=0;

        for (loop = 0; loop < 8; loop++)
        {
                // shift the result to get it ready for the next bit
                result >>= 1;
                // if result is one, then set MS bit
                if (OWReadBit())
                        result |= 0x80;
        }
        return result;
}

//-----------------------------------------------------------------------------
// Write a 1-Wire data byte and return the sampled result.
//
int OWTouchByte(int data)
{
        int loop, result=0;

        for (loop = 0; loop < 8; loop++)
        {
                // shift the result to get it ready for the next bit
                result >>= 1;

                // If sending a '1' then read a bit else write a '0'
                if (data & 0x01)
                {
                        if (OWReadBit())
                                result |= 0x80;
                }
                else
                        OWWriteBit(0);

                // shift the data byte for the next bit
                data >>= 1;
        }
        return result;
}

//-----------------------------------------------------------------------------
// Write a block 1-Wire data bytes and return the sampled result in the same
// buffer.
//
void OWBlock(unsigned char *data, int data_len)
{
        int loop;

        for (loop = 0; loop < data_len; loop++)
        {
                data[loop] = OWTouchByte(data[loop]);
        }
}

//-----------------------------------------------------------------------------
// Set all devices on 1-Wire to overdrive speed. Return '1' if at least one
// overdrive capable device is detected.
//
int OWOverdriveSkip(unsigned char *data, int data_len)
{
        // set the speed to 'standard'
        SetSpeed(1);

        // reset all devices
        if (OWTouchReset()) // Reset the 1-Wire bus
                return 0; // Return if no devices found

        // overdrive skip command
        OWWriteByte(0x3C);

        // set the speed to 'overdrive'
        SetSpeed(0);

        // do a 1-Wire reset in 'overdrive' and return presence result
        return OWTouchReset();
}

// </editor-fold>


#ifdef	__cplusplus
}
#endif

#endif	/* FRAMEWORK_1WIRE_H */