/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef PIR_H
#define	PIR_H

#include <xc.h>  

#define WARM_UP_DELAY_ms        (5000U) //Initial wait time to settle PIR output 
#define PIR_DEFAULT_SENSITIVITY (15U)  //Chnage in ADC count required to report object detection. Lesser the number higher is the sensitivity
#define DAC_VALUE_853mV         (107U) //DAC data register value corresponding to 853 mV
#define LED_BLINK_PERIOD_ms     (250U)

/**
 * @brief Setup PIR sensor
 *      1. Sets DAC output. 
 *      2. Configure ADC positive channel
 *      3. Warmup the sensor
 * @param None
 * @return None
 */
void PIR_Setup(void);

/**
 * @brief Wait for PIR output to stabilize
 *      1. Provides delay to settle the PIR output. 
 *      2. Sets the PIR sensitivity  (ADCC threshold) according to PIR output 
 *          observed after the warmup delay
 * @param None
 * @return None
 */
void PIR_WarmUp(void);

/**
 * @brief Sets PIR sensitivity
 *  Sets ADCC lower threshold depending upon sensitivity value.
 *  Lesser the number higher is the sensitivity.
 * @param None
 * @return None
 */
void PIR_SetSensitivity(uint8_t newSensitivity);

/**
 * @brief Stops ADCC sampling of PIR output
 * Stops of TMR2 (ADCC triggering)
 * @param None
 * @return None
 */
void PIR_StopSampling(void);

/**
 * @brief Starts ADCC sampling of PIR output
 * Starts of TMR2 (ADCC triggering)
 * @param None
 * @return None
 */
void PIR_StartSampling(void);

/**
 * @brief Enable ADCC threshold interrupt
 */
void PIR_StartObservingOutputChange(void);

/**
 * @brief Disable ADCC threshold interrupt
 */
void PIR_StopObservingOutputChange(void);

#endif	/* PIR_H */
