/*
© [2022] Microchip Technology Inc. and its subsidiaries.

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
#include "../mcc_generated_files/system/system.h"
#include "pir.h"

#define WARM_UP_DELAY_ms        (5000U) //Initial wait time to settle PIR output 
#define PIR_DEFAULT_SENSITIVITY (15U)  //Change in ADC count required to report object detection. Lesser the number higher is the sensitivity
#define DAC_VALUE_853mV         (91U) //DAC data register value corresponding to 853 mV
#define LED_BLINK_PERIOD_ms     (250U)

adc_result_t PIR_outputatNormalCondition; //PIR output when there is no movement

/**
 * @brief Setup PIR sensor
 *      1. Sets DAC output. 
 *      2. Configure ADC positive channel
 *      3. Warmup the sensor
 * @param None
 * @return None
 */
void PIR_Setup(void)
{
    DAC2_SetOutput(DAC_VALUE_853mV); //DAC is connected to OPA non-inverting terminal.

    ADPCH = channel_OPA1OUT; //Set OPA output as ADC positive channel

    PIR_WarmUp();
}

/**
 * @brief Wait for PIR output to stabilize
 *      1. Provides delay to settle the PIR output. 
 *      2. Sets the PIR sensitivity  (ADCC threshold) according to PIR output 
 *          observed after the warmup delay
 * @param None
 * @return None
 */
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
    
    ADCC_SetUpperThreshold(0U);

    PIR_SetSensitivity(PIR_DEFAULT_SENSITIVITY);
}

/**
 * @brief Enable ADCC threshold interrupt
 */
void PIR_StartObservingOutputChange(void)
{
    PIR6bits.ADTIF = 0; //Clear the ADC Threshold interrupt flag
    PIE6bits.ADTIE = 1; //Enabling ADCC threshold interrupt
}

/**
 * @brief Disable ADCC threshold interrupt
 */
void PIR_StopObservingOutputChange(void)
{
    PIE6bits.ADTIE = 0; //Disable ADCC threshold interrupt
    PIR6bits.ADTIF = 0; //Clear the ADC Threshold interrupt flag
}

/**
 * @brief Starts ADCC sampling of PIR output
 *      Starts of TMR2 (ADCC triggering)
 * @param None
 * @return None
 */
void PIR_StartSampling(void)
{
    Timer2.Start(); //TMR2 is used as ADC trigger source
}

/**
 * @brief Stops ADCC sampling of PIR output
 *      Stops of TMR2 (ADCC triggering)
 * @param None
 * @return None
 */
void PIR_StopSampling(void)
{
    Timer2.Stop(); //TMR2 is used as ADC trigger source
}

/**
 * @brief Sets PIR sensitivity
 *      Sets ADCC lower threshold depending upon sensitivity value.
 *      Lesser the number higher is the sensitivity.
 * @param None
 * @return None
 */
void PIR_SetSensitivity(uint8_t newSensitivity)
{
    printf("Setting sensitivity : %d\r\n", newSensitivity * -1);
    ADCC_SetLowerThreshold(newSensitivity * -1); //set threshold as per sensitivity. 
    //ADCC ERROR value will be negative when ADCC Filtered output is less than defined setpoint 
}





