/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.0.0
*/

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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA4 aliases
#define RN4871_RST_N_TRIS                 TRISAbits.TRISA4
#define RN4871_RST_N_LAT                  LATAbits.LATA4
#define RN4871_RST_N_PORT                 PORTAbits.RA4
#define RN4871_RST_N_WPU                  WPUAbits.WPUA4
#define RN4871_RST_N_OD                   ODCONAbits.ODCA4
#define RN4871_RST_N_ANS                  ANSELAbits.ANSA4
#define RN4871_RST_N_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define RN4871_RST_N_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define RN4871_RST_N_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define RN4871_RST_N_GetValue()           PORTAbits.RA4
#define RN4871_RST_N_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define RN4871_RST_N_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define RN4871_RST_N_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define RN4871_RST_N_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define RN4871_RST_N_SetPushPull()        do { ODCONAbits.ODCA4 = 0; } while(0)
#define RN4871_RST_N_SetOpenDrain()       do { ODCONAbits.ODCA4 = 1; } while(0)
#define RN4871_RST_N_SetAnalogMode()      do { ANSELAbits.ANSA4 = 1; } while(0)
#define RN4871_RST_N_SetDigitalMode()     do { ANSELAbits.ANSA4 = 0; } while(0)

// get/set IO_RB5 aliases
#define IO_RB5_TRIS                 TRISBbits.TRISB5
#define IO_RB5_LAT                  LATBbits.LATB5
#define IO_RB5_PORT                 PORTBbits.RB5
#define IO_RB5_WPU                  WPUBbits.WPUB5
#define IO_RB5_OD                   ODCONBbits.ODCB5
#define IO_RB5_ANS                  ANSELBbits.ANSB5
#define IO_RB5_SetHigh()            do { LATBbits.LATB5 = 1; } while(0)
#define IO_RB5_SetLow()             do { LATBbits.LATB5 = 0; } while(0)
#define IO_RB5_Toggle()             do { LATBbits.LATB5 = ~LATBbits.LATB5; } while(0)
#define IO_RB5_GetValue()           PORTBbits.RB5
#define IO_RB5_SetDigitalInput()    do { TRISBbits.TRISB5 = 1; } while(0)
#define IO_RB5_SetDigitalOutput()   do { TRISBbits.TRISB5 = 0; } while(0)
#define IO_RB5_SetPullup()          do { WPUBbits.WPUB5 = 1; } while(0)
#define IO_RB5_ResetPullup()        do { WPUBbits.WPUB5 = 0; } while(0)
#define IO_RB5_SetPushPull()        do { ODCONBbits.ODCB5 = 0; } while(0)
#define IO_RB5_SetOpenDrain()       do { ODCONBbits.ODCB5 = 1; } while(0)
#define IO_RB5_SetAnalogMode()      do { ANSELBbits.ANSB5 = 1; } while(0)
#define IO_RB5_SetDigitalMode()     do { ANSELBbits.ANSB5 = 0; } while(0)

// get/set IO_RB7 aliases
#define IO_RB7_TRIS                 TRISBbits.TRISB7
#define IO_RB7_LAT                  LATBbits.LATB7
#define IO_RB7_PORT                 PORTBbits.RB7
#define IO_RB7_WPU                  WPUBbits.WPUB7
#define IO_RB7_OD                   ODCONBbits.ODCB7
#define IO_RB7_ANS                  ANSELBbits.ANSB7
#define IO_RB7_SetHigh()            do { LATBbits.LATB7 = 1; } while(0)
#define IO_RB7_SetLow()             do { LATBbits.LATB7 = 0; } while(0)
#define IO_RB7_Toggle()             do { LATBbits.LATB7 = ~LATBbits.LATB7; } while(0)
#define IO_RB7_GetValue()           PORTBbits.RB7
#define IO_RB7_SetDigitalInput()    do { TRISBbits.TRISB7 = 1; } while(0)
#define IO_RB7_SetDigitalOutput()   do { TRISBbits.TRISB7 = 0; } while(0)
#define IO_RB7_SetPullup()          do { WPUBbits.WPUB7 = 1; } while(0)
#define IO_RB7_ResetPullup()        do { WPUBbits.WPUB7 = 0; } while(0)
#define IO_RB7_SetPushPull()        do { ODCONBbits.ODCB7 = 0; } while(0)
#define IO_RB7_SetOpenDrain()       do { ODCONBbits.ODCB7 = 1; } while(0)
#define IO_RB7_SetAnalogMode()      do { ANSELBbits.ANSB7 = 1; } while(0)
#define IO_RB7_SetDigitalMode()     do { ANSELBbits.ANSB7 = 0; } while(0)

// get/set IO_RC1 aliases
#define LED0_TRIS                 TRISCbits.TRISC1
#define LED0_LAT                  LATCbits.LATC1
#define LED0_PORT                 PORTCbits.RC1
#define LED0_WPU                  WPUCbits.WPUC1
#define LED0_OD                   ODCONCbits.ODCC1
#define LED0_ANS                  ANSELCbits.ANSC1
#define LED0_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define LED0_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define LED0_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define LED0_GetValue()           PORTCbits.RC1
#define LED0_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define LED0_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define LED0_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define LED0_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define LED0_SetPushPull()        do { ODCONCbits.ODCC1 = 0; } while(0)
#define LED0_SetOpenDrain()       do { ODCONCbits.ODCC1 = 1; } while(0)
#define LED0_SetAnalogMode()      do { ANSELCbits.ANSC1 = 1; } while(0)
#define LED0_SetDigitalMode()     do { ANSELCbits.ANSC1 = 0; } while(0)

// get/set IO_RC2 aliases
#define IO_RC2_TRIS                 TRISCbits.TRISC2
#define IO_RC2_LAT                  LATCbits.LATC2
#define IO_RC2_PORT                 PORTCbits.RC2
#define IO_RC2_WPU                  WPUCbits.WPUC2
#define IO_RC2_OD                   ODCONCbits.ODCC2
#define IO_RC2_ANS                  ANSELCbits.ANSC2
#define IO_RC2_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define IO_RC2_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define IO_RC2_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define IO_RC2_GetValue()           PORTCbits.RC2
#define IO_RC2_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define IO_RC2_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define IO_RC2_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define IO_RC2_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define IO_RC2_SetPushPull()        do { ODCONCbits.ODCC2 = 0; } while(0)
#define IO_RC2_SetOpenDrain()       do { ODCONCbits.ODCC2 = 1; } while(0)
#define IO_RC2_SetAnalogMode()      do { ANSELCbits.ANSC2 = 1; } while(0)
#define IO_RC2_SetDigitalMode()     do { ANSELCbits.ANSC2 = 0; } while(0)

// get/set IO_RC3 aliases
#define RC3_IN1_TRIS                 TRISCbits.TRISC3
#define RC3_IN1_LAT                  LATCbits.LATC3
#define RC3_IN1_PORT                 PORTCbits.RC3
#define RC3_IN1_WPU                  WPUCbits.WPUC3
#define RC3_IN1_OD                   ODCONCbits.ODCC3
#define RC3_IN1_ANS                  ANSELCbits.ANSC3
#define RC3_IN1_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define RC3_IN1_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define RC3_IN1_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define RC3_IN1_GetValue()           PORTCbits.RC3
#define RC3_IN1_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define RC3_IN1_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define RC3_IN1_SetPullup()          do { WPUCbits.WPUC3 = 1; } while(0)
#define RC3_IN1_ResetPullup()        do { WPUCbits.WPUC3 = 0; } while(0)
#define RC3_IN1_SetPushPull()        do { ODCONCbits.ODCC3 = 0; } while(0)
#define RC3_IN1_SetOpenDrain()       do { ODCONCbits.ODCC3 = 1; } while(0)
#define RC3_IN1_SetAnalogMode()      do { ANSELCbits.ANSC3 = 1; } while(0)
#define RC3_IN1_SetDigitalMode()     do { ANSELCbits.ANSC3 = 0; } while(0)

// get/set IO_RC4 aliases
#define IO_RC4_TRIS                 TRISCbits.TRISC4
#define IO_RC4_LAT                  LATCbits.LATC4
#define IO_RC4_PORT                 PORTCbits.RC4
#define IO_RC4_WPU                  WPUCbits.WPUC4
#define IO_RC4_OD                   ODCONCbits.ODCC4
#define IO_RC4_ANS                  ANSELCbits.ANSC4
#define IO_RC4_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define IO_RC4_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define IO_RC4_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define IO_RC4_GetValue()           PORTCbits.RC4
#define IO_RC4_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define IO_RC4_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define IO_RC4_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define IO_RC4_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define IO_RC4_SetPushPull()        do { ODCONCbits.ODCC4 = 0; } while(0)
#define IO_RC4_SetOpenDrain()       do { ODCONCbits.ODCC4 = 1; } while(0)
#define IO_RC4_SetAnalogMode()      do { ANSELCbits.ANSC4 = 1; } while(0)
#define IO_RC4_SetDigitalMode()     do { ANSELCbits.ANSC4 = 0; } while(0)

// get/set IO_RC5 aliases
#define IO_RC5_TRIS                 TRISCbits.TRISC5
#define IO_RC5_LAT                  LATCbits.LATC5
#define IO_RC5_PORT                 PORTCbits.RC5
#define IO_RC5_WPU                  WPUCbits.WPUC5
#define IO_RC5_OD                   ODCONCbits.ODCC5
#define IO_RC5_ANS                  ANSELCbits.ANSC5
#define IO_RC5_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define IO_RC5_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define IO_RC5_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define IO_RC5_GetValue()           PORTCbits.RC5
#define IO_RC5_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define IO_RC5_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define IO_RC5_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define IO_RC5_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define IO_RC5_SetPushPull()        do { ODCONCbits.ODCC5 = 0; } while(0)
#define IO_RC5_SetOpenDrain()       do { ODCONCbits.ODCC5 = 1; } while(0)
#define IO_RC5_SetAnalogMode()      do { ANSELCbits.ANSC5 = 1; } while(0)
#define IO_RC5_SetDigitalMode()     do { ANSELCbits.ANSC5 = 0; } while(0)

// get/set IO_RC6 aliases
#define RN4871_UART_TX_IND_TRIS                 TRISCbits.TRISC6
#define RN4871_UART_TX_IND_LAT                  LATCbits.LATC6
#define RN4871_UART_TX_IND_PORT                 PORTCbits.RC6
#define RN4871_UART_TX_IND_WPU                  WPUCbits.WPUC6
#define RN4871_UART_TX_IND_OD                   ODCONCbits.ODCC6
#define RN4871_UART_TX_IND_ANS                  ANSELCbits.ANSC6
#define RN4871_UART_TX_IND_SetHigh()            do { LATCbits.LATC6 = 1; } while(0)
#define RN4871_UART_TX_IND_SetLow()             do { LATCbits.LATC6 = 0; } while(0)
#define RN4871_UART_TX_IND_Toggle()             do { LATCbits.LATC6 = ~LATCbits.LATC6; } while(0)
#define RN4871_UART_TX_IND_GetValue()           PORTCbits.RC6
#define RN4871_UART_TX_IND_SetDigitalInput()    do { TRISCbits.TRISC6 = 1; } while(0)
#define RN4871_UART_TX_IND_SetDigitalOutput()   do { TRISCbits.TRISC6 = 0; } while(0)
#define RN4871_UART_TX_IND_SetPullup()          do { WPUCbits.WPUC6 = 1; } while(0)
#define RN4871_UART_TX_IND_ResetPullup()        do { WPUCbits.WPUC6 = 0; } while(0)
#define RN4871_UART_TX_IND_SetPushPull()        do { ODCONCbits.ODCC6 = 0; } while(0)
#define RN4871_UART_TX_IND_SetOpenDrain()       do { ODCONCbits.ODCC6 = 1; } while(0)
#define RN4871_UART_TX_IND_SetAnalogMode()      do { ANSELCbits.ANSC6 = 1; } while(0)
#define RN4871_UART_TX_IND_SetDigitalMode()     do { ANSELCbits.ANSC6 = 0; } while(0)

/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handler for the RC6 pin functionality
 * @param none
 * @return none
 */
void RC6_ISR(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for RC6 pin interrupt-on-change functionality.
 *        Allows selecting an interrupt handler for RC6 at application runtime.
 * @pre Pins intializer called
 * @param InterruptHandler function pointer.
 * @return none
 */
void RC6_SetInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup  pinsdriver
 * @brief Dynamic Interrupt Handler for RC6 pin.
 *        This is a dynamic interrupt handler to be used together with the RC6_SetInterruptHandler() method.
 *        This handler is called every time the RC6 ISR is executed and allows any function to be registered at runtime.
 * @pre Pins intializer called
 * @param none
 * @return none
 */
extern void (*RC6_InterruptHandler)(void);

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for RC6 pin. 
 *        This is a predefined interrupt handler to be used together with the RC6_SetInterruptHandler() method.
 *        This handler is called every time the RC6 ISR is executed. 
 * @pre Pins intializer called
 * @param none
 * @return none
 */
void RC6_DefaultInterruptHandler(void);


#endif // PINS_H
/**
 End of File
*/