#include <xc.h>
#include "mcc_generated_files/system/system.h"
#include "pir_sensor.h"

adc_result_t PIR_outputatNormalCondition; //PIR output when there is no movement

void PIR_Setup(void)
{
    DAC2_SetOutput(DAC_VALUE_853mV); //DAC is connected to OPA non-inverting terminal.
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

    PIR_outputatNormalCondition = ADC1_GetFilterValue();
    
    printf("PIR output at normal condition : %u\r\n", PIR_outputatNormalCondition);
    
    ADC1_DefineSetPoint(PIR_outputatNormalCondition);
    
    ADC1_SetUpperThreshold(PIR_DEFAULT_SENSITIVITY);

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
    ADC1_SetLowerThreshold(newSensitivity * -1); //set threshold as per sensitivity. 
    //ADCC ERROR value will be negative when ADCC Filtered output is less than defined setpoint 
}





