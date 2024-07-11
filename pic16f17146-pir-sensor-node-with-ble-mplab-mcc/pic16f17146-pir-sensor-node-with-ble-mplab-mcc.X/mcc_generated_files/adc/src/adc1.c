/**
 * ADC1 Generated Driver File
 * 
 * @file adc1.c
 * 
 * @ingroup  adc1
 * 
 * @brief This file contains the API implementations for the ADC1 driver.
 *
 * @version ADC1 Driver Version 1.0.3
*/
/*
© [2024] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/

#include <xc.h>
#include "../adc1.h"

static void (*ADC1_ADTI_InterruptHandler)(void);
static void ADC1_DefaultADTI_ISR(void);

/**
  Section: ADCC Module APIs
*/

void ADC1_Initialize(void)
{
    // set the ADC1 to the options selected in the User Interface
    //ADLTHL 0; 
    ADLTHL = 0x0;
    //ADLTHH 0; 
    ADLTHH = 0x0;
    //ADUTHL 0; 
    ADUTHL = 0x0;
    //ADUTHH 0; 
    ADUTHH = 0x0;
    //ADSTPTL 0; 
    ADSTPTL = 0x0;
    //ADSTPTH 0; 
    ADSTPTH = 0x0;
    //ADACCU 0x0; 
    ADACCU = 0x0;
    //ADRPT 0; 
    ADRPT = 0x00;
    //ADCHS ANC2; 
    ADPCH = 0x12;
    //ADCHS ANA0; 
    ADNCH = 0x0;
    //ADACQL 3; 
    ADACQL = 0x3;
    //ADACQH 0; 
    ADACQH = 0x0;
    //ADCAP Additional uC disabled; 
    ADCAP = 0x0;
    //ADPREL 0; 
    ADPREL = 0x0;
    //ADPREH 0; 
    ADPREH = 0x0;
    //CGA0 disabled; CGA1 disabled; CGA2 disabled; CGA4 disabled; CGA5 disabled; 
    ADCG1A = 0x0;
    //CGB4 disabled; CGB5 disabled; CGB6 disabled; CGB7 disabled; 
    ADCG1B = 0x0;
    //CGC0 disabled; CGC1 disabled; CGC2 disabled; CGC3 disabled; CGC4 disabled; CGC5 disabled; CGC6 disabled; CGC7 disabled; 
    ADCG1C = 0x0;
    //ADDSEN disabled; ADPCSC internal sampling capacitor and ext i/o pin; ADGPOL digital_low; ADIPEN disabled; ADPPOL Vss; 
    ADCON1 = 0x0;
    //ADMD Low_pass_filter_mode; ADACLR disabled; ADCRS 6; ADPSIS RES; 
    ADCON2 = 0x64;
    //ADTMD ADERR < ADLTH; ADSOI ADGO not cleared; ADCALC Filtered value vs setpoint; 
    ADCON3 = 0x51;
    //ADMATH registers not updated; 
    ADSTAT = 0x0;
    //ADPREF VDD; 
    ADREF = 0x0;
    //ADACT TMR2; 
    ADACT = 0x4;
    //ADCCS FOSC/2; 
    ADCLK = 0x0;
    //GO_nDONE undefined; ADIC single-ended mode; ADFM right justified; ADCS ADCRC; ADCONT disabled; ADON enabled; 
    ADCON0 = 0x94;
    
    //Clear the ADC interrupt flag
    PIR6bits.ADIF = 0;

    //Clear the ADC Threshold interrupt flag
    PIR6bits.ADTIF = 0;
    //Configure interrupt handlers
    ADC1_SetADTIInterruptHandler(ADC1_DefaultADTI_ISR);
    //Enable ADC1 threshold interrupt.
    PIE6bits.ADTIE = 1;
}
void ADC1_StartConversion(adc1_channel_t channel)
{
    //Selects the A/D channel
    ADPCH = channel;      
  
    //Starts the conversion
    ADCON0bits.GO = 1;
}

bool ADC1_IsConversionDone(void)
{
    //Starts the conversion
    return ((unsigned char)(!ADCON0bits.GO));
}

adc_result_t ADC1_GetConversionResult(void)
{
    //Returns the result
    return ((adc_result_t)((ADRESH << 8) + ADRESL));
}

adc_result_t ADC1_GetSingleConversion(adc1_channel_t channel)
{
    //Selects the A/D channel
    ADPCH = channel;  
   
    //Disables the continuous mode.
    ADCON0bits.CONT = 0;

    //Starts the conversion
    ADCON0bits.GO = 1;

    //Waits for the conversion to finish
    while (ADCON0bits.GO)
    {
    }
    
    //Conversion finished, returns the result
    return ((adc_result_t)((ADRESH << 8) + ADRESL));
}

inline void ADC1_StopConversion(void)
{
    //Resets the ADGO bit.
    ADCON0bits.GO = 0;
}

inline void ADC1_SetStopOnInterrupt(void)
{
    //Sets the ADSOI bit.
    ADCON3bits.ADSOI = 1;
}

inline void ADC1_DischargeSampleCapacitor(void)
{
    //Sets the ADC channel to AVss.
    ADPCH = 0x3a;   
}

void ADC1_LoadAcquisitionRegister(uint16_t acquisitionValue)
{
    //Loads the ADACQH and ADACQL registers.
    ADACQH = (uint8_t) (acquisitionValue >> 8);
    ADACQL = (uint8_t) acquisitionValue;  
}

void ADC1_SetPrechargeTime(uint16_t prechargeTime)
{
    //Loads the ADPREH and ADPREL registers.
    ADPREH = (uint8_t) (prechargeTime >> 8);
    ADPREL = (uint8_t) prechargeTime;
}

void ADC1_SetRepeatCount(uint8_t repeatCount)
{
    //Loads the ADRPT register.
    ADRPT = repeatCount;   
}

uint8_t ADC1_GetCurrentCountofConversions(void)
{
    //Returns the contents of ADCNT register
    return ADCNT;
}

inline void ADC1_ClearAccumulator(void)
{
    //Resets the ADCON2bits.ADACLR bit.
    ADCON2bits.ADACLR = 1;
}

uint24_t ADC1_GetAccumulatorValue(void)
{
    //Returns the contents of ADACCU, ADACCH and ADACCL registers
    return (((uint24_t)ADACCU << 16)+((uint24_t)ADACCH << 8) + ADACCL);
}

void ADC1_DefineSetPoint(uint16_t setPoint)
{
    //Sets the ADSTPTH and ADSTPTL registers
    ADSTPTH = (uint8_t) (setPoint >> 8);
    ADSTPTL = (uint8_t) setPoint;
}

uint16_t ADC1_GetErrorCalculation(void)
{
    //Returns the contents of ADERRH and ADERRL registers
    return ((uint16_t)((ADERRH << 8) + ADERRL));
}

void ADC1_SetUpperThreshold(uint16_t upperThreshold)
{
    //Sets the ADUTHH and ADUTHL registers
    ADUTHH = (uint8_t) (upperThreshold >> 8);
    ADUTHL = (uint8_t) upperThreshold;
}

void ADC1_SetLowerThreshold(uint16_t lowerThreshold)
{
    //Sets the ADLTHH and ADLTHL registers
    ADLTHH = (uint8_t) (lowerThreshold >> 8);
    ADLTHL = (uint8_t) lowerThreshold;
}

uint16_t ADC1_GetFilterValue(void)
{
    //Returns the contents of ADFLTRH and ADFLTRL registers
    return ((uint16_t)((ADFLTRH << 8) + ADFLTRL));
}

uint16_t ADC1_GetPreviousResult(void)
{
    //Returns the contents of ADPREVH and ADPREVL registers
    return ((uint16_t)((ADPREVH << 8) + ADPREVL));
}

bool ADC1_HasAccumulatorOverflowed(void)
{
    //Returns the status of ADSTATbits.ADAOV
    return ADSTATbits.ADAOV;
}

inline void ADC1_EnableDoubleSampling(void)
{
    //Sets the ADCON1bits.ADDSEN
    ADCON1bits.ADDSEN = 1;
}

inline void ADC1_EnableContinuousConversion(void)
{
    //Sets the ADCON0bits.CONT
    ADCON0bits.CONT = 1;
}

inline void ADC1_DisableContinuousConversion(void)
{
    //Resets the ADCON0bits.CONT
    ADCON0bits.CONT = 0;
}

bool ADC1_HasErrorCrossedUpperThreshold(void)
{
    //Returns the value of ADSTATbits.ADUTHR bit.
    return ADSTATbits.ADUTHR;
}

bool ADC1_HasErrorCrossedLowerThreshold(void)
{
    //Returns the value of ADSTATbits.ADLTHR bit.
    return ADSTATbits.ADLTHR;
}

uint8_t ADC1_GetConversionStageStatus(void)
{
    //Returns the contents of ADSTATbits.ADSTAT field.
    return ADSTATbits.ADSTAT;
}


void ADC1_ThresholdISR(void)
{
    //Clears the ADCC Threshold interrupt flag
    PIR6bits.ADTIF = 0;

    if (ADC1_ADTI_InterruptHandler != NULL)
    {
        ADC1_ADTI_InterruptHandler();
    }
}

void ADC1_SetADTIInterruptHandler(void (* InterruptHandler)(void))
{
    ADC1_ADTI_InterruptHandler = InterruptHandler;
}

static void ADC1_DefaultADTI_ISR(void)
{
    //Add your interrupt code here or
    //Use ADC1_SetADTIInterruptHandler() function to use Custom ISR
}
