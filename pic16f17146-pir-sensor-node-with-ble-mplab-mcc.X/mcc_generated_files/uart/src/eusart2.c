/**
 * EUSART2 Generated Driver API Header File
 * 
 * @file eusart2.c
 * 
 * @ingroup eusart2
 * 
 * @brief This is the generated driver implementation file for the EUSART2 driver using the Enhanced Universal Synchronous and Asynchronous Receiver Transceiver (EUSART) module.
 *
 * @version EUSART2 Driver Version 3.0.1
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

/**
  Section: Included Files
*/
#include "../eusart2.h"

/**
  Section: Macro Declarations
*/

#define EUSART2_RX_BUFFER_SIZE (8U) //buffer size should be 2^n
#define EUSART2_RX_BUFFER_MASK (EUSART2_RX_BUFFER_SIZE - 1U)

/**
  Section: Driver Interface
 */

const uart_drv_interface_t UART2 = {
    .Initialize = &EUSART2_Initialize,
    .Deinitialize = &EUSART2_Deinitialize,
    .Read = &EUSART2_Read,
    .Write = &EUSART2_Write,
    .IsRxReady = &EUSART2_IsRxReady,
    .IsTxReady = &EUSART2_IsTxReady,
    .IsTxDone = &EUSART2_IsTxDone,
    .TransmitEnable = &EUSART2_TransmitEnable,
    .TransmitDisable = &EUSART2_TransmitDisable,
    .AutoBaudSet = &EUSART2_AutoBaudSet,
    .AutoBaudQuery = &EUSART2_AutoBaudQuery,
    .BRGCountSet = NULL,
    .BRGCountGet = NULL,
    .BaudRateSet = NULL,
    .BaudRateGet = NULL,
    .AutoBaudEventEnableGet = NULL,
    .ErrorGet = &EUSART2_ErrorGet,
    .TxCompleteCallbackRegister = NULL,
    .RxCompleteCallbackRegister = &EUSART2_RxCompleteCallbackRegister,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &EUSART2_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &EUSART2_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = NULL,
    .EventCallbackRegister = NULL,
};

/**
  Section: EUSART2 variables
*/
static volatile uint8_t eusart2RxHead = 0;
static volatile uint8_t eusart2RxTail = 0;
static volatile uint8_t eusart2RxBuffer[EUSART2_RX_BUFFER_SIZE];
static volatile eusart2_status_t eusart2RxStatusBuffer[EUSART2_RX_BUFFER_SIZE];
static volatile uint8_t eusart2RxCount;

static volatile eusart2_status_t eusart2RxLastError;

/**
  Section: EUSART2 APIs
*/

void (*EUSART2_RxInterruptHandler)(void);
static void (*EUSART2_RxCompleteInterruptHandler)(void) = NULL;

static void (*EUSART2_FramingErrorHandler)(void) = NULL;
static void (*EUSART2_OverrunErrorHandler)(void) = NULL;

static void EUSART2_DefaultFramingErrorCallback(void);
static void EUSART2_DefaultOverrunErrorCallback(void);

void EUSART2_ReceiveISR(void);


/**
  Section: EUSART2  APIs
*/

void EUSART2_Initialize(void)
{
    PIE5bits.RC2IE = 0;   
     EUSART2_RxInterruptHandler = EUSART2_ReceiveISR;   

    // Set the EUSART2 module to the options selected in the user interface.

    //ABDEN disabled; WUE disabled; BRG16 16bit_generator; SCKP Non-Inverted; 
    BAUD2CON = 0x48; 
    //ADDEN disabled; CREN enabled; SREN disabled; RX9 8-bit; SPEN enabled; 
    RC2STA = 0x90; 
    //TX9D 0x0; BRGH hi_speed; SENDB sync_break_complete; SYNC asynchronous; TXEN enabled; TX9 8-bit; CSRC client; 
    TX2STA = 0x26; 
    //SPBRGL 25; 
    SP2BRGL = 0x19; 
    //SPBRGH 0; 
    SP2BRGH = 0x0; 

    EUSART2_FramingErrorCallbackRegister(EUSART2_DefaultFramingErrorCallback);
    EUSART2_OverrunErrorCallbackRegister(EUSART2_DefaultOverrunErrorCallback);
    eusart2RxLastError.status = 0;  

    eusart2RxHead = 0;
    eusart2RxTail = 0;
    eusart2RxCount = 0;

    PIE5bits.RC2IE = 1; 
}

void EUSART2_Deinitialize(void)
{
    PIE5bits.RC2IE = 0;    
    BAUD2CON = 0x00;
    RC2STA = 0x00;
    TX2STA = 0x00;
    SP2BRGL = 0x00;
    SP2BRGH = 0x00;
}

void EUSART2_Enable(void)
{
    RC2STAbits.SPEN = 1;

}

void EUSART2_Disable(void)
{
    RC2STAbits.SPEN = 0;
}


void EUSART2_TransmitEnable(void)
{
    TX2STAbits.TXEN = 1;
}

void EUSART2_TransmitDisable(void)
{
    TX2STAbits.TXEN = 0;
}

void EUSART2_ReceiveEnable(void)
{
    RC2STAbits.CREN = 1;
}

void EUSART2_ReceiveDisable(void)
{
    RC2STAbits.CREN = 0;
}

void EUSART2_SendBreakControlEnable(void)
{
    TX2STAbits.SENDB = 1;
}

void EUSART2_SendBreakControlDisable(void)
{
    TX2STAbits.SENDB = 0;
}

void EUSART2_AutoBaudSet(bool enable)
{
    if(enable)
    {
        BAUD2CONbits.ABDEN = 1;
    }
    else
    {
       BAUD2CONbits.ABDEN = 0; 
    }
}

bool EUSART2_AutoBaudQuery(void)
{
return (bool)(!BAUD2CONbits.ABDEN);
}

bool EUSART2_IsAutoBaudDetectOverflow(void)
{
    return (bool)BAUD2CONbits.ABDOVF; 
}

void EUSART2_AutoBaudDetectOverflowReset(void)
{
    BAUD2CONbits.ABDOVF = 0; 
}

void EUSART2_ReceiveInterruptEnable(void)
{
    PIE5bits.RC2IE = 1;
}
void EUSART2_ReceiveInterruptDisable(void)
{
    PIE5bits.RC2IE = 0; 
}

bool EUSART2_IsRxReady(void)
{
    return (eusart2RxCount ? true : false);
}

bool EUSART2_IsTxReady(void)
{
    return (bool)(PIR5bits.TX2IF && TX2STAbits.TXEN);
}

bool EUSART2_IsTxDone(void)
{
    return TX2STAbits.TRMT;
}

size_t EUSART2_ErrorGet(void)
{
    eusart2RxLastError.status = eusart2RxStatusBuffer[(eusart2RxTail + 1U) & EUSART2_RX_BUFFER_MASK].status;
    return eusart2RxLastError.status;
}

uint8_t EUSART2_Read(void)
{
    uint8_t readValue  = 0;
    uint8_t tempRxTail;
    
    readValue = eusart2RxBuffer[eusart2RxTail];

    tempRxTail = (eusart2RxTail + 1U) & EUSART2_RX_BUFFER_MASK; // Buffer size of RX should be in the 2^n
    
    eusart2RxTail = tempRxTail;

    eusart2RxLastError = eusart2RxStatusBuffer[eusart2RxTail];
    

    PIE5bits.RC2IE = 0; 
    if(0U != eusart2RxCount)
    {
        eusart2RxCount--;
    }
    PIE5bits.RC2IE = 1;
    return readValue;
}

void EUSART2_ReceiveISR(void)
{
    uint8_t regValue;
	uint8_t tempRxHead;

    // use this default receive interrupt handler code
    eusart2RxStatusBuffer[eusart2RxHead].status = 0;

    if(true == RC2STAbits.OERR)
    {
        eusart2RxStatusBuffer[eusart2RxHead].oerr = 1;
        if(NULL != EUSART2_OverrunErrorHandler)
        {
            EUSART2_OverrunErrorHandler();
        }   
    }   
    if(true == RC2STAbits.FERR)
    {
        eusart2RxStatusBuffer[eusart2RxHead].ferr = 1;
        if(NULL != EUSART2_FramingErrorHandler)
        {
            EUSART2_FramingErrorHandler();
        }   
    } 
    
    regValue = RC2REG;
    
    tempRxHead = (eusart2RxHead + 1U) & EUSART2_RX_BUFFER_MASK;// Buffer size of RX should be in the 2^n
    if (tempRxHead == eusart2RxTail) 
    {
		// ERROR! Receive buffer overflow 
	} 
    else
    {
        eusart2RxBuffer[eusart2RxHead] = regValue;
		eusart2RxHead = tempRxHead;
		eusart2RxCount++;
	}   

    if(NULL != EUSART2_RxCompleteInterruptHandler)
    {
        (*EUSART2_RxCompleteInterruptHandler)();
    } 
}

void EUSART2_Write(uint8_t txData)
{
    TX2REG = txData;
}

static void EUSART2_DefaultFramingErrorCallback(void)
{
    
}

static void EUSART2_DefaultOverrunErrorCallback(void)
{
    //Continuous Receive must be cleared to clear Overrun Error else Rx will not receive upcoming bytes
    RC2STAbits.CREN = 0;
    RC2STAbits.CREN = 1;
}

void EUSART2_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART2_FramingErrorHandler = callbackHandler;
    }
}

void EUSART2_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART2_OverrunErrorHandler = callbackHandler;
    }    
}

void EUSART2_RxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       EUSART2_RxCompleteInterruptHandler = callbackHandler; 
    }   
}

