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

#include <xc.h>
#include "mcc_generated_files/system/system.h"
#include "pir_sensor.h"

adc_result_t PIR_outputatNormalCondition; //PIR output when there is no movement

void PIR_Setup(void)
{
    ADPCH = channel_ANC2; // ADC positive channel is selected to be RC2
    DAC2_SetOutput(DAC_VALUE_853mV); //DAC output is connected to OPA non-inverting terminal.
    PIR_WarmUp();
}

void PIR_WarmUp(void)
{
    uint16_t ledBlinkTime = 0;
    PIR_StopObservingOutputChange();

    PIR_StartSampling();

    //Wait for PIR output to stabilize. Meanwhile blink LED
    while(ledBlinkTime < WARM_UP_DELAY_ms)
    {
        LED0_Toggle();
        __delay_ms(LED_BLINK_PERIOD_ms);
        ledBlinkTime += LED_BLINK_PERIOD_ms;
    }
    
    LED0_SetHigh();  //turn off LED

    PIR_outputatNormalCondition = ADCC_GetFilterValue();
    
    printf("PIR output at normal condition : %u\r\n", PIR_outputatNormalCondition);
    
    ADCC_DefineSetPoint(PIR_outputatNormalCondition);

    PIR_SetSensitivity(PIR_DEFAULT_SENSITIVITY);
}

void PIR_StartObservingOutputChange(void)
{
    PIR6bits.ADTIF = 0; //Clear the ADC Threshold interrupt flag
    PIE6bits.ADTIE = 1; //Enabling ADCC threshold interrupt
}

void PIR_StopObservingOutputChange(void)
{
    PIE6bits.ADTIE = 0; //Disable ADCC threshold interrupt
    PIR6bits.ADTIF = 0; //Clear the ADC Threshold interrupt flag
}

void PIR_StartSampling(void)
{
    Timer2.Start(); //TMR2 is used as ADC trigger source
}

void PIR_StopSampling(void)
{
    Timer2.Stop(); //TMR2 is used as ADC trigger source
}

void PIR_SetSensitivity(uint8_t newSensitivity)
{
    printf("Setting sensitivity : %d\r\n", newSensitivity * -1);
    ADCC_SetLowerThreshold(newSensitivity * -1); //set threshold as per sensitivity. 
    //ADCC ERROR value will be negative when ADCC Filtered output is less than defined setpoint 
}





