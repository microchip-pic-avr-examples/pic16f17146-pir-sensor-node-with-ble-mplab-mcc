/**
 * OPA1 Generated Driver API Header File.
 * 
 * @file opa1.h
 * 
 * @defgroup  opa1 OPA1
 * 
 * @brief This file contains the API prototypes and the related data structures for the OPA1 driver.
 *
 * @version OPA1 Driver Version 2.1.1
*/
/*
� [2024] Microchip Technology Inc. and its subsidiaries.

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

#ifndef OPA1_H
#define OPA1_H

/**
  Section: Included Files
*/
#include <stdint.h>
#include <stdbool.h>

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_Initialize API
 */
#define OPA1_Initialize OPA1_Initialize

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_EnableChargePump API
 */
#define OPA1_EnableChargePump OPA1_EnableChargePump

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_DisableChargePump API
 */
#define OPA1_DisableChargePump OPA1_DisableChargePump

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_EnableSoftwareUnityGain API
 */
#define OPA1_EnableSoftwareUnityGain OPA1_EnableSoftwareUnityGain

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_DisableSoftwareUnityGain API
 */
#define OPA1_DisableSoftwareUnityGain OPA1_DisableSoftwareUnityGain

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetPositiveChannel API
 */
#define OPA1_SetPositiveChannel OPA1_SetPositiveChannel

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetPositiveSource API
 */
#define OPA1_SetPositiveSource OPA1_SetPositiveSource

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetNegativeChannel API
 */
#define OPA1_SetNegativeChannel OPA1_SetNegativeChannel

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetNegativeSource API
 */
#define OPA1_SetNegativeSource OPA1_SetNegativeSource

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetResistorLadder API
 */
#define OPA1_SetResistorLadder OPA1_SetResistorLadder

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_EnableHardwareOverride API
 */
#define OPA1_EnableHardwareOverride OPA1_EnableHardwareOverride

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetHardwareOverrideSource API
 */
#define OPA1_SetHardwareOverrideSource OPA1_SetHardwareOverrideSource

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_DisableHardwareOverride API
 */
#define OPA1_DisableHardwareOverride OPA1_DisableHardwareOverride

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetSoftwareOverride API
 */
#define OPA1_SetSoftwareOverride OPA1_SetSoftwareOverride

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_SetInputOffset API
 */
#define OPA1_SetInputOffset OPA1_SetInputOffset

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_resistor_select
 */
#define OPA1_resistor_select OPA1_resistor_select

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_posChannel_select
 */
#define OPA1_posChannel_select OPA1_posChannel_select

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_posSource_select
 */
#define OPA1_posSource_select OPA1_posSource_select

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_negChannel_select
 */
#define OPA1_negChannel_select OPA1_negChannel_select

/**
 * @ingroup  opa1
 * @brief    This macro defines the Custom Name for \ref OPA1_negSource_select
 */
#define OPA1_negSource_select OPA1_negSource_select

/**
 * @ingroup opa1
 * @enum OPA1_resistor_select
 * @brief Enumeration of the R2/R1 resistor ratio values.
*/
typedef enum
{
    OPA1_R2byR1_is_0dp07,    /**<R2/R1 = 0.07*/
    OPA1_R2byR1_is_0dp14,    /**<R2/R1 = 0.14*/
    OPA1_R2byR1_is_0dp33,    /**<R2/R1 = 0.33*/
    OPA1_R2byR1_is_1,        /**<R2/R1 = 1*/
    OPA1_R2byR1_is_1dp67,    /**<R2/R1 = 1.67*/
    OPA1_R2byR1_is_3,        /**<R2/R1 = 3*/
    OPA1_R2byR1_is_7,        /**<R2/R1 = 7*/
    OPA1_R2byR1_is_15        /**<R2/R1 = 15*/
} OPA1_resistor_select;

/**
 * @ingroup opa1
 * @enum OPA1_posChannel_select
 * @brief Enumeration of the available positive channels.
*/
typedef enum
{
    OPA1_posChannel_Vss,
    OPA1_posChannel_GSEL,
    OPA1_posChannel_OPA1IN,
    OPA1_posChannel_Vdd_by_2,
    OPA1_posChannel_DAC1,
    OPA1_posChannel_DAC2
} OPA1_posChannel_select;

/**
 * @ingroup opa1
 * @enum OPA1_posSource_select
 * @brief Enumeration of the available positive sources.
*/
typedef enum
{
    OPA1_RB5_IN0_pos = 0x0,
    OPA1_RC3_IN1_pos = 0x1,
    OPA1_RA2_IN2_pos = 0x2,
    OPA1_RA0_IN3_pos = 0x3
} OPA1_posSource_select;

/**
 * @ingroup opa1
 * @enum OPA1_negChannel_select
 * @brief Enumeration of the available negative channels.
*/
typedef enum
{
    OPA1_negChannel_No_Connection = 0x0,
    OPA1_negChannel_GSEL = 0x1,
    OPA1_negChannel_OPA1IN = 0x2,
    OPA1_negChannel_DAC1 = 0x4,
    OPA1_negChannel_DAC2 = 0x5
} OPA1_negChannel_select;

/**
 * @ingroup opa1
 * @enum OPA1_negSource_select
 * @brief Enumeration of the available negative sources.
*/
typedef enum
{
    OPA1_RB4_IN0_neg = 0x0,   
    OPA1_RC3_IN1_neg = 0x1,   
    OPA1_RA2_IN2_neg = 0x2,   
    OPA1_RA0_IN3_neg = 0x3,   
    OPA1_Vss = 0x7   
} OPA1_negSource_select;

/**
 * @ingroup opa1
 * @brief Initializes the OPA1 module. This is called only once before calling other OPA1 APIs.
 * @param None.
 * @return None.
*/
void OPA1_Initialize(void);

/**
 * @ingroup opa1
 * @brief Enables the OPA1 charge pump.
 * @pre OPA1_Initialize() is already called.
 * @param None.
 * @return None.
*/
inline void OPA1_EnableChargePump(void);

/**
 * @ingroup opa1
 * @brief Disables the OPA1 charge pump.
 * @pre OPA1_Initialize() is already called.
 * @param None.
 * @return None.
*/
inline void OPA1_DisableChargePump(void);

/**
 * @ingroup opa1
 * @brief Enables the OPA1 to operate with unity gain.
 * @pre OPA1_Initialize() is already called.
 * @param None.
 * @return None.
*/
inline void OPA1_EnableSoftwareUnityGain(void);

/**
 * @ingroup opa1
 * @brief Disables unity gain for OPA1. The inverting input is connected to the designated OPAxIN- pin.
 * @pre OPA1_Initialize() is already called.
 * @param None.
 * @return None.
*/
inline void OPA1_DisableSoftwareUnityGain(void);

/**
 * @ingroup opa1
 * @brief Sets the positive channel.
 * @pre OPA1_Initialize() is already called.
 * @param posChannel - Selected positive channel for OPA1. Refer to the OPA1_posChannel_select for a complete list of possible values.
 * @return None.
*/
inline void OPA1_SetPositiveChannel(OPA1_posChannel_select posChannel);

/**
 * @ingroup opa1
 * @brief Sets the positive source.
 * @pre OPA1_Initialize() is already called.
 * @param posSource - Selected positive source for OPA1. Refer to OPA1_posSource_select for a complete list of possible values.
 * @return None.
*/
inline void OPA1_SetPositiveSource(OPA1_posSource_select posSource);

/**
 * @ingroup opa1
 * @brief Sets the negative channel.
 * @pre OPA1_Initialize() is already called.
 * @param Selected negative channel for OPA1. Refer to the OPA1_negChannel_select for a complete list of possible values.
 * @return None.
*/
inline void OPA1_SetNegativeChannel(OPA1_negChannel_select negChannel);

/**
 * @ingroup opa1
 * @brief Sets the negative source.
 * @pre OPA1_Initialize() is already called.
 * @param Selected negative source for OPA1. Refer to the OPA1_negSource_select for a complete list of possible values.
 * @return None.
*/
inline void OPA1_SetNegativeSource(OPA1_negSource_select negSource);

/**
 * @ingroup opa1
 * @brief Sets the R1 and R2 values of internal resistor ladder.
 * @pre OPA1_Initialize() is already called.
 * @param resistorSelection - Selected R2/R1 resistor ratio for OPA1. Refer to the OPA1_resistor_select for a complete list of possible values.
 * @return None.
*/
void OPA1_SetResistorLadder(OPA1_resistor_select resistorSelection);

/**
 * @ingroup opa1
 * @brief Enables hardware override control.
 * @pre OPA1_Initialize() is already called.
 * @param None.
 * @return None.
*/
inline void OPA1_EnableHardwareOverride(void);

/**
 * @ingroup opa1
 * @brief Selects the hardware override source and polarity.
 * @pre The OPA1_EnableHardwareOverride() is already called.
 * @param overrideSource - Selected output override source for OPA1.
 * @param polarity - Selected hardware control input polarity for OPA1.
 * @return None.
*/
void OPA1_SetHardwareOverrideSource(uint8_t overrideSource, uint8_t polarity);

/**
 * @ingroup opa1
 * @brief Disables the hardware override control.
 * @pre OPA1_Initialize() is already called.
 * @param None.
 * @return None.
*/
inline void OPA1_DisableHardwareOverride(void);

/**
 * @ingroup opa1
 * @brief Selects the software override mode.
 * @pre The OPA1_DisableHardwareOverride() is already called.
 * @param softwareControl - Two-bit value representing the selected Software Control mode.
 * @return None.
*/
inline void OPA1_SetSoftwareOverride(uint8_t softwareControl);

/**
 * @ingroup opa1
 * @brief Sets the input offset calibration value of OPA1.
 * @pre OPA1_Initialize() is already called.
 * @param offset - Input offset calibration value.
 * @return None.
*/
inline void OPA1_SetInputOffset(uint8_t offset);

#endif //OPA1_H