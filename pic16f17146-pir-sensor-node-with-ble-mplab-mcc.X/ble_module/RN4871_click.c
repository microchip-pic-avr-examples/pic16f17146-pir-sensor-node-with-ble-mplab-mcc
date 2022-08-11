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
#include <string.h>
#include "RN4871_click.h"
#include "../mcc_generated_files/system/system.h"

void RN4871_CaptureReceivedMessage(void);

volatile uint16_t RN4871_bufferIndex = 0;
volatile uint8_t RN4871_buffer[RN4871_BUFFER_SIZE]; //array to stored data received from BLE

/**
 * @brief Send one byte to RN4871
 * @param byte - The byte to send to the RN487
 * @return None
 */
void RN4871_SendByte(uint8_t byte)
{
    while (!(UART2.IsTxReady()));
    UART2.Write(byte);
}

/**
 * @brief Send buffer with mentioned length to RN4871
 * @param buffer - Address of the buffer
 * @param length- Length of buffer
 * @return None
 */
void RN4871_SendBuffer(const char *buffer, uint8_t length)
{
    while (length--)
    {
        RN4871_SendByte(*buffer++);
    }
}

/**
 * @brief Send string to RN4871
 * @param data - Address of the string
 * @return None
 */
void RN4871_SendString(const char* data)
{
    RN4871_SendBuffer(data, (uint8_t) strlen(data));
}

/**
 * @brief Setup the RN4871
 *      1. Set UART2 RX interrupt handler
 *      2. Reset the module
 * @param None
 * @return None
 */
void RN4871_Setup(void)
{
    UART2.RxCompleteCallbackRegister(RN4871_CaptureReceivedMessage);
    RN4871_ResetModule();
    RN4871_ClearReceivedMessage();
}

/**
 * @brief Reset the RN4871
 *      1. Hardware reset the module using RST_N pin
 * @param None
 * @return None
 */
void RN4871_ResetModule(void)
{
    RN4871_RST_N_SetLow(); // Reset using GPIO
    RN4871_blockingWait(1);
    RN4871_RST_N_SetHigh();
    RN4871_blockingWait(5);
}

/**
 * @brief Clear the received message buffer
 * @param None
 * @return None
 */
void RN4871_ClearReceivedMessage(void)
{
    memset((void *) RN4871_buffer, 0, RN4871_BUFFER_SIZE);
    RN4871_bufferIndex = 0;
}

/**
 * @brief Check response received from BLE module
 *      1. Compares response with received message buffer
 * @param resonse - Address of string to check
 * @retval true - If expected response is received
 * @retval false - If expected response is not received
 */
eRN4871_Responses_t RN4871_CheckResponse(const char *response)
{
    eRN4871_Responses_t ret = NO_RESPONSE;

    if (strstr((const char *) RN4871_buffer, response))
    {
        ret = OK;
    }

    return ret;
}

/**
 * @brief Wait by calling delay function
 * @param limit - Time to wait 
 * @return None
 */
void RN4871_blockingWait(uint16_t limit)
{
    for (uint16_t counter = 0; counter < limit; counter++)
    {
        __delay_ms(15);
    }
}

/**
 * @brief Send data to BLE module and wait for response
 *      1. Send command to BLE module
 *      2. Wait till receive correct response
 * @param sendString - Address of string to send
 * @param response - Address of response string to check
 * @param delay - Time to wait for response 
 * @return None
 */
void RN4871_SendAndWait(const char *sendString, const char *response, uint16_t delay)
{
    do
    {
        RN4871_ClearReceivedMessage();
        RN4871_SendString(sendString);
        RN4871_blockingWait(delay);
    }
    while (RN4871_CheckResponse(response) == NO_RESPONSE);
}

/**
 * @brief Sends status of RX buffer
 *      1. Store received data from UART2 into the buffer
 * @param None
 * @retval true - If buffer any data is received
 * @retval false - If buffer no data is received
 */
bool RN4871_DataReceived(void)
{
    return (RN4871_bufferIndex ? true : false);
}

/**
 * @brief User Interrupt handler for UART2 RX interrupt
 *      1. Store received data from UART2 into the buffer
 * @param None
 * @return None
 */
void RN4871_CaptureReceivedMessage(void)
{
    uint8_t data;
    data = RC2REG;

    if (RN4871_bufferIndex < RN4871_BUFFER_SIZE)
    {
        RN4871_buffer[RN4871_bufferIndex++] = data;
    }
}

/**
 * @brief Enable interrupt on change for RN4871_UART_TX_IND pin.
 *      RN4871_UART_TX_IND pin goes low before RN4871 sends data.
 * @param None
 * @return None
 */
void RN4871_StartObservingIncomingData(void)
{
    IOCCFbits.IOCCF7 = 0; //Clear RN4871_UART_TX_IND pin interrupt on change flag
    IOCCNbits.IOCCN7 = 1; //enable interrupt on change for RN4871_UART_TX_IND pin 
}

/**
 * @brief Enable interrupt on change for RN4871_UART_TX_IND pin.
 *      RN4871_UART_TX_IND pin goes low before RN4871 sends data.
 * @param None
 * @return None
 */
void RN4871_StopObservingIncomingData(void)
{
    IOCCNbits.IOCCN7 = 0; //disable interrupt on change for RN4871_UART_TX_IND pin 
}




