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

void RN4871_Setup(void);
void RN4871_ResetModule(void);
void RN4871_blockingWait(uint16_t limit);
void RN4871_SendAndWait(const char *sendString, const char *response, uint16_t delay);
eRN4871_Responses_t RN4871_CheckResponse(const char *response);
void RN4871_ClearReceivedMessage(void);
void RN4871_SendString(const char *);
void RN4871_SendBuffer(const char *buffer, uint8_t length);
void RN4871_SendByte(uint8_t byte);
bool RN4871_DataReceived(void);
void RN4871_StartObservingIncomingData(void);
void RN4871_StopObservingIncomingData(void);

#endif	/* RN4871_CLICK_H */

