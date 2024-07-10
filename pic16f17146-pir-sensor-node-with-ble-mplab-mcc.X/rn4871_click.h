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
#ifndef RN4871_CLICK_H
#define	RN4871_CLICK_H

#include <xc.h>  
#include <stdint.h>
#include <stdbool.h>

typedef enum 
{ 
    NO_RESPONSE = 0, 
    OK 
} eRN4871_Responses_t;

#define RN4871_BUFFER_SIZE (128U)

extern volatile uint16_t RN4871_bufferIndex;
extern volatile uint8_t  RN4871_buffer[RN4871_BUFFER_SIZE];

/**
 * @brief Setup the RN4871
 *   1. Set UART2 RX interrupt handler
 *   2. Reset the module
 * @param None
 * @return None
 */
void RN4871_Setup(void);

/**
 * @brief Reset the RN4871
 *      1. Hardware reset the module using RST_N pin
 * @param None
 * @return None
 */
void RN4871_ResetModule(void);

/**
 * @brief Wait by calling delay function
 * @param limit - Time to wait 
 * @return None
 */
void RN4871_blockingWait(uint16_t limit);

/**
 * @brief Send data to BLE module and wait for response
 *      1. Send command to BLE module
 *      2. Wait till receive correct response
 * @param sendString - Address of string to send
 * @param response - Address of response string to check
 * @param delay - Time to wait for response 
 * @return None
 */
void RN4871_SendAndWait(const char *sendString, const char *response, uint16_t delay);

/**
 * @brief Check response received from BLE module
 *      1. Compares response with received message buffer
 * @param resonse - Address of string to check
 * @retval true - If expected response is received
 * @retval false - If expected response is not received
 */
eRN4871_Responses_t RN4871_CheckResponse(const char *response);

/**
 * @brief Clear the received message buffer
 * @param None
 * @return None
 */
void RN4871_ClearReceivedMessage(void);

/**
 * @brief Send string to RN4871
 * @param data - Address of the string
 * @return None
 */
void RN4871_SendString(const char *);

/**
 * @brief Send buffer with mentioned length to RN4871
 * @param buffer - Address of the buffer
 * @param length- Length of buffer
 * @return None
 */
void RN4871_SendBuffer(const char *buffer, uint8_t length);

/**
 * @brief Send one byte to RN4871
 * @param byte - The byte to send to the RN487
 * @return None
 */
void RN4871_SendByte(uint8_t byte);

/**
 * @brief Sends status of RX buffer
 *      1. Store received data from UART2 into the buffer
 * @param None
 * @retval true - If buffer any data is received
 * @retval false - If buffer no data is received
 */
bool RN4871_DataReceived(void);

/**
 * @brief Enable interrupt on change for RN4871_UART_TX_IND pin.
 *      RN4871_UART_TX_IND pin goes low before RN4871 sends data.
 * @param None
 * @return None
 */
void RN4871_StartObservingIncomingData(void);

/**
 * @brief Enable interrupt on change for RN4871_UART_TX_IND pin.
 *      RN4871_UART_TX_IND pin goes low before RN4871 sends data.
 * @param None
 * @return None
 */
void RN4871_StopObservingIncomingData(void);

/**
 * @brief User Interrupt handler for UART2 RX interrupt
 *      1. Store received data from UART2 into the buffer
 * @param None
 * @return None
 */
void RN4871_CaptureReceivedMessage(void);

#endif	/* RN4871_CLICK_H */
