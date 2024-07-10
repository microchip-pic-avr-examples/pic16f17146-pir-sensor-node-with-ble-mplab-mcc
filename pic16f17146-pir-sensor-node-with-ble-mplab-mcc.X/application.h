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
#ifndef APPLICATION_H
#define	APPLICATION_H

#include <xc.h> 

#define WAIT_TIME_FOR_PIR_OP_SETTLE_ms (5000U) //Duration of wait when movement is detected (can be changed according to application need)
#define PIR_MAX_SENSITIVITY (25U) //Change in ADC count required to report object detection. Lesser the number higher is the sensitivity 
#define TIEM_TO_RECEIVE_UART_DATA_ms (50U) //time to receive approx 45 UART characters after UART_TX_IND_IOC interrupt
#define ADCTMD_ADERR_LESS_THAN_ADLTH (0b001)
#define ADCTMD_ADERR_GREATHER_THAN_ADUTH (0b110)

/**
 * @brief Setup Application    
 *      1. Initialize sleep mode
 *      2. Configure interrupt handlers for ADCC Threshold and RN4871_UART_TX_IND pin IOC.
 *      3. RN4871 and transparent UART service setup  
 *      4. Setup PIR Sensor 
 * @param None
 * @return None
 */
void ApplicationSetup(void);

/**
 * @brief Application Task. Runs in while(1) loop
 *      1. Checks for data reception from BLE module and takes action accordingly. 
 *      2. Send data over BLE whenever movement is detected. 
 *      3. Puts microcontroller in sleep mode. Microcontroller wakes up when   
 *         data is received from BLE module or movement is detected
 * @param None
 * @return None
 */
void ApplicationTask(void);

/**
 * @brief Process data received from BLE module
 *      1. Check if valid data is received
 *      2. Set sensitivity or starts warmup depending upon data received
 *      3. Notify "action taken" or "data validity" over BLE depending on data received
 * @param None
 * @return None
 */
void ProcessReceivedDataFromBLE(void); 

/**
 * @brief This functions specifies actions to be taken if movement is detected.
 * @param None
 * @return None
 */
void setAlarm(void); 

/**
 * @brief Turns on LED0
 * @param None
 * @return None
 */
inline void LED0_ON(void);

/**
 * @brief Turns off LED0
 * @param None
 * @return None
 */
inline void LED0_OFF(void);

/**
 * @brief User Interrupt handler for IOC for RN4871_UART_TX_IND pin
 *      Interrupt occurs when BLE module is about to send data over UART
 * @param None
 * @return None
 */
void RN4871_UART_TX_IND_IOC_UserInteruptHandler(void);

/**
 * @brief ADCC threshold interrupt callback function
 * @param None
 * @return None
 */
void ADCC_UserThresholdInterrupt(void);

/**
 * @brief Timer 0 interrupt callback function
 * @param None
 * @return None
 */
void TMR0_UserInterruptHandler(void);
  

#endif	/* APPLICATION_H */
