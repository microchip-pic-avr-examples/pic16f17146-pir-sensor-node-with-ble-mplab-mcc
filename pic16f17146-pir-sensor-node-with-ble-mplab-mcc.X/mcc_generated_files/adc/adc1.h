/**
 * ADC1 Generated Driver API Header File
 * 
 * @file adc1.h
 * 
 * @defgroup  adc1 ADC1
 * 
 * @brief This file contains the API prototypes and data types for the ADC1 driver.
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

#ifndef ADC1_H
#define ADC1_H

/**
  Section: Included Files
*/

#include <stdint.h>
#include <stdbool.h>

/**
 @ingroup adc1
 @typedef adc_result_t
 @brief Used for the result of the Analog-to-Digital (A/D) conversion.
*/
typedef uint16_t adc_result_t;
#ifndef uint24_t
typedef __uint24 uint24_t;
#endif


/**
 * @ingroup adc1
 * @enum adc1_channel_t
 * @brief Contains the available ADC channels.
*/
typedef enum
{
    channel_ADCG1 =  0x3,
    channel_OPA1OUT =  0x39,
    channel_VSS =  0x3a,
    channel_Temp =  0x3b,
    channel_DAC1OUT =  0x3c,
    channel_DAC2OUT =  0x3d,
    channel_FVR_Buffer1 =  0x3e,
    channel_FVR_Buffer2 =  0x3f,
    channel_ANC2 =  0x12
} adc1_channel_t;

/**
  Section: ADC1 Module APIs
*/

/**
 * @ingroup adc1
 * @brief Initializes the ADC module. This routine is called before any other ADC routine.
 * @param None.
 * @return None.
*/
void ADC1_Initialize(void);

/**
 * @ingroup adc1
 * @brief Starts A/D conversion on the selected channel.
 * @pre ADC1_Initialize}() function is called before calling this function to enable ADC.
 * @param channel - Analog channel number on which the A/D conversion has to be applied.
 *                  Refer to adc1_channel_t for the available list of channels.
 * @return None.
*/
void ADC1_StartConversion(adc1_channel_t channel);

/**
 * @ingroup adc1
 * @brief Checks if ongoing A/D conversion is complete.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @retval True - A/D conversion is complete.
 * @retval False - A/D conversion is ongoing.
*/
bool ADC1_IsConversionDone(void);

/**
 * @ingroup adc1
 * @brief Retrieves the result of the latest A/D conversion.
 * @pre ADC1_StartConversion() is called before calling this function and completion status is checked using the ADC1_IsConversionDone routine.
 * @param None.
 * @return The result of A/D conversion. Refer to the adc_result_t.
*/
adc_result_t ADC1_GetConversionResult(void);

/**
 * @ingroup adc1
 * @brief Retrieves the result of a single A/D conversion on any given channel.
 * @pre Enable ADC using the ADC1_Initialize}() function. Also ADC1_DisableContinuousConversion() is called before calling this function.
 * @param channel - Analog channel number on which the A/D conversion will be applied.
 *                  Refer to adc1_channel_t  for the available channels.
 * @return The result of a single A/D conversion. Refer to the adc_result_t.
*/
adc_result_t ADC1_GetSingleConversion(adc1_channel_t channel);

/**
 * @ingroup adc1
 * @brief Stops the ongoing A/D conversion.
 * @pre ADC1_StartConversion() function is called before calling this function.
 * @param None.
 * @return None.
*/
inline void ADC1_StopConversion(void);

/**
 * @ingroup adc1
 * @brief Enables Stop On Interrupt bit.
 * @pre ADC1_EnableContinuousConversion() function is called before calling this function.
 * @param None.
 * @return None.
*/
inline void ADC1_SetStopOnInterrupt(void);

/**
 * @ingroup adc1
 * @brief Discharges the input sample capacitor by setting the channel to AVss.
 * @param None.
 * @return None.
*/
inline void ADC1_DischargeSampleCapacitor(void);

/**
 * @ingroup adc1
 * @brief Loads ADC Acquisition Time Control register with the specified value.
 * @param acquisitionValue - Value to be loaded in the acquisition time control register.
 * @return None.
*/
void ADC1_LoadAcquisitionRegister(uint16_t acquisitionValue);

/**
 * @ingroup adc1
 * @brief Loads ADC Precharge Time Control register with the specified value.
 * @param prechargeTime - Value to be loaded in the Precharge Time Control register.
 * @return None.
*/
void ADC1_SetPrechargeTime(uint16_t prechargeTime);

/**
 * @ingroup adc1
 * @brief Loads ADC Repeat Counter register with specified value.
 * @param repeatCount - Value to be loaded to ADC Repeat Counter register.
 * @return None.
*/
void ADC1_SetRepeatCount(uint8_t repeatCount);

/**
 * @ingroup adc1
 * @brief Retrieves the current value of ADC Repeat Count register.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @return Current value of the ADC Repeat Count register
*/
uint8_t ADC1_GetCurrentCountofConversions(void);

/**
 * @ingroup adc1
 * @brief Clears the accumulator.
 * @param None.
 * @return None.
*/
inline void ADC1_ClearAccumulator(void);

/**
 * @ingroup adc1
 * @brief Retrieves the value of ADC accumulator.
 * @param None.
 * @return Value of the ADC accumulator.
*/
uint24_t ADC1_GetAccumulatorValue(void);

/**
 * @ingroup adc1
 * @brief Determines whether ADC accumulator has overflowed.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @retval True - ADC accumulator has overflowed.
 * @retval False - ADC accumulator has not overflowed.
*/
bool ADC1_HasAccumulatorOverflowed(void);

/**
 * @ingroup adc1
 * @brief Retrieves the value of the ADC Filter(ADFLTR) register.
 * @param None.
 * @return 16-bit value obtained from the high byte ADFLTR(ADFLTRH) and low byte ADFLTR(ADFLTRL) registers.
*/
uint16_t ADC1_GetFilterValue(void);

/**
 * @ingroup adc1
 * @brief Retrieves the value of the ADC Previous(ADPREV) register.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @return 16-bit value obtained from the high byte ADPREV(ADPREVH) and low byte ADPREV(ADPREVL) registers.
*/
uint16_t ADC1_GetPreviousResult(void);

/**
 * @ingroup adc1
 * @brief Sets the value of the ADC Threshold Setpoint(ADSTPT) register.
 * @param setPoint - 16-bit value for the ADC Threshold Setpoint register.
 * @return None.
*/
void ADC1_DefineSetPoint(uint16_t setPoint);

/**
 * @ingroup adc1
 * @brief Sets the value of ADC Upper Threshold(ADUTH) register.
 * @param upperThreshold - 16-bit value for the ADC Upper Threshold register.
 * @return None.
*/
void ADC1_SetUpperThreshold(uint16_t upperThreshold);

/**
 * @ingroup adc1
 * @brief Sets the value of ADC Lower Threshold(ADLTH) register.
 * @param lowerThreshold - 16-bit value for the ADC Lower Threshold register.
 * @return None.
*/
void ADC1_SetLowerThreshold(uint16_t lowerThreshold);

/**
 * @ingroup adc1
 * @brief Retrieves the value of ADC Set-point Error register.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @return 16-bit value obtained from the high byte ADERR(ADERRH) and low byte ADERR(ADERRL) registers.
*/
uint16_t ADC1_GetErrorCalculation(void);

/**
 * @ingroup adc1
 * @brief Enables double-sampling bit.
 * @param None.
 * @return None.
*/
inline void ADC1_EnableDoubleSampling(void);

/**
 * @ingroup adc1
 * @brief Enables continuous conversion.
 * @param None.
 * @return None.
*/
inline void ADC1_EnableContinuousConversion(void);

/**
 * @ingroup adc1
 * @brief Disables continuous conversion.
 * @param None.
 * @return None.
*/
inline void ADC1_DisableContinuousConversion(void);

/**
 * @ingroup adc1
 * @brief Determines if ADC error has crossed the upper threshold.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @retval True - ADC error has crossed the upper threshold.
 * @retval False - ADC error has not crossed the upper threshold.
*/
bool ADC1_HasErrorCrossedUpperThreshold(void);

/**
 * @ingroup adc1
 * @brief Determines if ADC error is less than the lower threshold.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @retval True - ADC error has crossed the lower threshold.
 * @retval False - ADC error has not crossed the lower threshold.
*/
bool ADC1_HasErrorCrossedLowerThreshold(void);

/**
 * @ingroup adc1
 * @brief Retrieves the multi-stage status.
 * @pre ADC1_StartConversion() is called before calling this function.
 * @param None.
 * @return Contents of ADC STATUS register.
*/
uint8_t ADC1_GetConversionStageStatus(void);

/**
 * @ingroup adc1
 * @brief Sets the callback for the ADC Threshold Interrupt(ADTI).
 * @param InterruptHandler - Callback Function to be called on the interrupt event
 * @return None.
*/
void ADC1_SetADTIInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup adc1
 * @brief Implements the ADC Threshold Interrupt(ADTI) service routine for the interrupt-driven implementations.
 * @param None.
 * @return None.
*/
void ADC1_ThresholdISR(void);

#endif//ADC1_H
