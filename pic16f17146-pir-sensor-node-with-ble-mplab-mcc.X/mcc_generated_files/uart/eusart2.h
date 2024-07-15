/**
 * EUSART2 Generated Driver API Header File
 * 
 * @file eusart2.h
 * 
 * @defgroup eusart2 EUSART2
 * 
 * @brief This file contains API prototypes and other datatypes for EUSART2 module.
 *
 * @version EUSART2 Driver Version 3.0.0
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

#ifndef EUSART2_H
#define EUSART2_H

/**
  Section: Included Files
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "../system/system.h"
#include "uart_drv_interface.h"

#ifdef __cplusplus  // Provide C++ Compatibility

    extern "C" {

#endif


#define UART2_interface UART2


#define UART2_Initialize     EUSART2_Initialize
#define UART2_Deinitialize   EUSART2_Deinitialize
#define UART2_Write          EUSART2_Write
#define UART2_Read           EUSART2_Read
#define UART2__IsRxReady     EUSART2_IsRxReady
#define UART2_IsTxReady      EUSART2_IsTxReady
#define UART2_IsTxDone       EUSART2_IsTxDone

#define UART2_TransmitEnable       EUSART2_TransmitEnable
#define UART2_TransmitDisable      EUSART2_TransmitDisable
#define UART2_AutoBaudSet          EUSART2_AutoBaudSet
#define UART2_AutoBaudQuery        EUSART2_AutoBaudQuery
#define UART2_BRGCountSet                (NULL)
#define UART2_BRGCountGet                (NULL)
#define UART2_BaudRateSet                (NULL)
#define UART2_BaudRateGet                (NULL)
#define UART2__AutoBaudEventEnableGet    (NULL)
#define UART2_ErrorGet             EUSART2_ErrorGet

#define UART2_TxCompleteCallbackRegister     (NULL)
#define UART2_RxCompleteCallbackRegister      EUSART2_RxCompleteCallbackRegister
#define UART2_TxCollisionCallbackRegister  (NULL)
#define UART2_FramingErrorCallbackRegister EUSART2_FramingErrorCallbackRegister
#define UART2_OverrunErrorCallbackRegister EUSART2_OverrunErrorCallbackRegister
#define UART2_ParityErrorCallbackRegister  (NULL)
#define UART2_EventCallbackRegister        (NULL)


/**
 @ingroup eusart2
 @struct eusart2_status_t
 @breif This is a structre defined for errors in reception of data.
 */
typedef union {
    struct {
        uint8_t perr : 1;     /**<This is a bit field for Parity Error status*/
        uint8_t ferr : 1;     /**<This is a bit field for Framing Error status*/
        uint8_t oerr : 1;     /**<This is a bit field for Overfrun Error status*/
        uint8_t reserved : 5; /**<Reserved*/
    };
    size_t status;            /**<Group byte for status errors*/
}eusart2_status_t;



/**
 Section: Data Type Definitions
 */

/**
 * @ingroup eusart2
 * @brief External object for eusart2_interface.
 */
extern const uart_drv_interface_t UART2;

/**
 * @ingroup eusart2
 * @brief This API initializes the EUSART2 driver.
 *        This routine initializes the EUSART2 module.
 *        This routine must be called before any other EUSART2 routine is called.
 *        This routine should only be called once during system initialization.
 * @param None.
 * @return None.
 */
void EUSART2_Initialize(void);

/**
 * @ingroup eusart2
 * @brief This API Deinitializes the EUSART2 driver.
 *        This routine disables the EUSART2 module.
 * @param None.
 * @return None.
 */
void EUSART2_Deinitialize(void);

/**
 * @ingroup eusart2
 * @brief This API enables the EUSART2 module.     
 * @param None.
 * @return None.
 */
inline void EUSART2_Enable(void);

/**
 * @ingroup eusart2
 * @brief This API disables the EUSART2 module.
 * @param None.
 * @return None.
 */
inline void EUSART2_Disable(void);

/**
 * @ingroup eusart2
 * @brief This API enables the EUSART2 transmitter.
 *        EUSART2 should also be enable to send bytes over TX pin.
 * @param None.
 * @return None.
 */
inline void EUSART2_TransmitEnable(void);

/**
 * @ingroup eusart2
 * @brief This API disables the EUSART2 transmitter.
 * @param None.
 * @return None.
 */
inline void EUSART2_TransmitDisable(void);

/**
 * @ingroup eusart2
 * @brief This API enables the EUSART2 Receiver.
 *        EUSART2 should also be enable to receive bytes over RX pin.
 * @param None.
 * @return None.
 */
inline void EUSART2_ReceiveEnable(void);

/**
 * @ingroup eusart2
 * @brief This API disables the EUSART2 Receiver.
 * @param None.
 * @return None.
 */
inline void EUSART2_ReceiveDisable(void);


/**
 * @ingroup eusart2
 * @brief This API enables the EUSART2 receiver interrupt.
 * @param None.
 * @return None.
 */
void EUSART2_ReceiveInterruptEnable(void);

/**
 * @ingroup eusart2
 * @brief This API disables the EUSART2 receiver interrupt.
 * @param None.
 * @return None.
 */
void EUSART2_ReceiveInterruptDisable(void);

/**
 * @ingroup eusart2
 * @brief This API enables the EUSART2 send break control.
 * @param None.
 * @return None.
 */
inline void EUSART2_SendBreakControlEnable(void);

/**
 * @ingroup eusart2
 * @brief This API disables the EUSART2 send break control.
 * @param None.
 * @return None.
 */
inline void EUSART2_SendBreakControlDisable(void);

/**
 * @ingroup eusart2
 * @brief This API enables the EUSART2 AutoBaud Detection.
 * @param bool enable.
 * @return None.
 */
inline void EUSART2_AutoBaudSet(bool enable);

/**
 * @ingroup eusart2
 * @brief This API reads the EUSART2 AutoBaud Detection Complete bit.
 * @param None.
 * @return bool.
 */
inline bool EUSART2_AutoBaudQuery(void);

/**
 * @ingroup eusart2
 * @brief This API reads the EUSART2 AutoBaud Detection overflow bit.
 * @param None.
 * @return None.
 */
inline bool EUSART2_IsAutoBaudDetectOverflow(void);

/**
 * @ingroup eusart2
 * @brief This API Reset the EUSART2 AutoBaud Detection Overflow bit.
 * @param None.
 * @return None.
 */
inline void EUSART2_AutoBaudDetectOverflowReset(void);

/**
 * @ingroup eusart2
 * @brief This API checks if EUSART2 receiver has received data and ready to be read.
 * @param None.
 * @retval true if EUSART2 receiver FIFO has a data
 * @retval false EUSART2 receiver FIFO is empty
 */
bool EUSART2_IsRxReady(void);

/**
 * @ingroup eusart2
 * @brief This function checks if EUSART2 transmitter is ready to accept a data byte.
 * @param None.
 * @retval true if EUSART2 transmitter FIFO has atleast 1 byte space
 * @retval false if EUSART2 transmitter FIFO is full
 */
bool EUSART2_IsTxReady(void);

/**
 * @ingroup eusart2
 * @brief This function return the status of transmit shift register (TSR).
 * @param None.
 * @retval true if Data completely shifted out from the TSR
 * @retval false if Data is present in Transmit FIFO and/or in TSR
 */
bool EUSART2_IsTxDone(void);

/**
 * @ingroup eusart2
 * @brief This function gets the error status of the last read byte.
 * @param None.
 * @return Status of the last read byte. See eusart2_status_t struct for more details.
 */
size_t EUSART2_ErrorGet(void);

/**
 * @ingroup eusart2
 * @brief This function reads the 8 bits from receiver FIFO register.
 * @pre The transfer status should be checked to see if the receiver is not empty
 *      before calling this function. EUSART2_IsRxReady() should be checked in if () before calling this API.
 * @param None.
 * @return 8-bit data from RX FIFO register.
 */
uint8_t EUSART2_Read(void);

/**
 * @ingroup eusart2
 * @brief This function writes a byte of data to the transmitter FIFO register.
 * @pre The transfer status should be checked to see if the transmitter is ready to accept a byte
 *      before calling this function. EUSART2_IsTxReady() should be checked in if() before calling this API.
 * @param txData  - Data byte to write to the TX FIFO.
 * @return None.
 */
void EUSART2_Write(uint8_t txData);

/**
 * @ingroup eusart2
 * @brief This API registers the function to be called upon framing error.
 * @param callbackHandler - a function pointer which will be called upon framing error condition.
 * @return None.
 */
void EUSART2_FramingErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup eusart2
 * @brief This API registers the function to be called upon overrun error.
 * @param callbackHandler - a function pointer which will be called upon overrun error condition.
 * @return None.
 */
void EUSART2_OverrunErrorCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup eusart2
 * @brief This is a pointer to the function that will be called upon receive interrupt.
 * @pre Initialize the EUSART2 module with receive interrupt enabled
 * @param None.
 * @return None.
 */
void (*EUSART2_RxInterruptHandler)(void);
/**
 * @ingroup eusart2
 * @brief This API registers the function to be called upon Receiver interrupt.
 * @param callbackHandler - a function pointer which will be called upon Receiver interrupt condition.
 * @return None.
 */
void EUSART2_RxCompleteCallbackRegister(void (* callbackHandler)(void));

/**
 * @ingroup eusart2
 * @brief This function is ISR function to be called upon Receiver interrupt.
 * @param void.
 * @return None.
 */
void EUSART2_ReceiveISR(void);

#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif  // EUSART2_H
