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
#include "mcc_generated_files/system/system.h"
#include "ble_module/RN4871_click.h"
#include "pir_sensor/pir.h"
#include "application.h"

#define WAIT_TIME_FOR_PIR_OP_SETTLE_ms (5000U) //Duration of wait when movement is detected (can be changed according to application need)
#define PIR_MAX_SENSITIVITY (25U) //Change in ADC count required to report object detection. Lesser the number higher is the sensitivity 
#define TIEM_TO_RECEIVE_UART_DATA_ms (50U) //time to receive approx 45 UART characters after UART_TX_IND_IOC interrupt
#define ADCTMD_ADERR_LESS_THAN_ADLTH (0b001)
#define ADCTMD_ADERR_GREATHER_THAN_ADUTH (0b110)

void ProcessReceivedDataFromBLE(void);
void setAlarm(void);
inline void LED0_ON(void);
inline void LED0_OFF(void);
void RN4871_UART_TX_IND_IOC_UserInteruptHandler(void);
void ADCC_UserThresholdInterrupt(void);
void TMR0_UserInterruptHandler(void);

volatile bool isDatacomingFromBLE = false; //flag to check if data is coming from RN4871
volatile bool isADCThresholdInterruptOccured = false;
bool isBLEConnected = false; //flag to check if BLE is connected 

/**
 * @brief Setup Application    
 *      1. Initialize sleep mode
 *      2. Configure interrupt handlers for ADCC Threshold and RN4871_UART_TX_IND pin IOC.
 *      3. RN4871 and transparent UART service setup  
 *      4. Setup PIR Sensor 
 * @param None
 * @return None
 */
void ApplicationSetup(void)
{
    CPUDOZEbits.IDLEN = 0; //clear idle enable bit so that MCU will go to sleep mode when SLEEP instruction is executed

    RN4871_StopObservingIncomingData();
    PIR_StopObservingOutputChange();

    RC6_SetInterruptHandler(RN4871_UART_TX_IND_IOC_UserInteruptHandler);
    ADCC_SetADTIInterruptHandler(ADCC_UserThresholdInterrupt);
    Timer0.TimeoutCallbackRegister(TMR0_UserInterruptHandler);

    printf("BLE : Setting up BLE Module..\r\n");
    RN4871_Setup();
    printf("BLE: BLE Setup Completed. Connect your Smartphone \r\n");
    RN4871_StartObservingIncomingData();

    printf("PIR: Setting up PIR sensor..\r\n");
    PIR_Setup();
    printf("PIR: PIR sensor setup completed\r\n");
    PIR_StartObservingOutputChange();
}

/**
 * @brief Application Task. Runs in while(1) loop
 *      1. Checks for data reception from BLE module and takes action accordingly. 
 *      2. Send data over BLE whenever movement is detected. 
 *      3. Puts microcontroller in sleep mode. Microcontroller wakes up when   
 *         data is received from BLE module or movement is detected
 * @param None
 * @return None
 */
void ApplicationTask(void)
{
    if (isDatacomingFromBLE)
    {
        RN4871_StopObservingIncomingData();

        isDatacomingFromBLE = false;

        __delay_ms(TIEM_TO_RECEIVE_UART_DATA_ms); //wait to receive all data

        if (RN4871_CheckResponse("%DISCONNECT%"))
        {
            isBLEConnected = false;
            printf("BLE : CONNECTION DROPPED\r\n");
        }
        else if (RN4871_CheckResponse("%STREAM_OPEN%"))
        {
            isBLEConnected = true;
            printf("BLE : STREAM HAS OPEN\r\n");
        }
        else
        {
            ProcessReceivedDataFromBLE();
        }

        RN4871_ClearReceivedMessage();

        RN4871_StartObservingIncomingData();
    }
    else if (isADCThresholdInterruptOccured)
    {
        printf("ADC Threshold Interrupt occurred");
        PIR_StopSampling();

        isADCThresholdInterruptOccured = false;      
        
        if(ADCC_HasErrorCrossedLowerThreshold())
        {
            //Movement detected 
            
            setAlarm();
            
            printf("ADC Threshold Interrupt set to : ADERR > ADUTH\r\n");

            //Configure ADC to Interrupt if ADERR > ADUTH. 
            //Interrupt will occur if there is no movement.
            ADCON3bits.TMD = ADCTMD_ADERR_GREATHER_THAN_ADUTH; 
        }
        else if (ADCC_HasErrorCrossedUpperThreshold())
        {
            //No movement 
           
            printf("ADC Threshold Interrupt set to : ADERR < ADLTH\r\n");

            //Configure ADC to Interrupt if ADERR < ADLTH. 
            //Interrupt will occur if there is movement.
            ADCON3bits.TMD = ADCTMD_ADERR_LESS_THAN_ADLTH;     
        }
        
        PIR_StartSampling();
    }
    else
    {
        printf("SLEEPING\r\n\n");

        while (!(UART1.IsTxDone())); //wait to finish printing messages
        while (!(UART2.IsTxDone())); //wait to finish sending data to BLE module

        SLEEP();
        NOP();

        //wake up can be due to IOC(incoming data from module) or
        // ADCC threshold interrupt (movement detection)

        printf("WOKE UP\r\n");
    }
}

/**
 * @brief Process data received from BLE module
 *      1. Check if valid data is received
 *      2. Set sensitivity or starts warmup depending upon data received
 *      3. Notify "action taken" or "data validity" over BLE depending on data received
 * @param None
 * @return None
 */
void ProcessReceivedDataFromBLE(void)
{
    uint8_t newPIRSensitivity = 0; //to store updated sensitivity

    if (RN4871_buffer[0] >= '0' && RN4871_buffer[0] <= '9')
    {
        newPIRSensitivity = PIR_MAX_SENSITIVITY - (RN4871_buffer[0] & 0x0F);

        PIR_SetSensitivity(newPIRSensitivity);

        RN4871_SendString("\r\n------------------------------------------\r\n");
        RN4871_SendString("Sensitivity updated ");
        RN4871_SendString("\r\n------------------------------------------\r\n");

        printf("PIR: NEW SENSITIVITY SET : %d \r\n", newPIRSensitivity);
    }
    else
    {
        printf("BLE: NO VALID DATA\r\n");

        RN4871_SendString("\r\n------------------------------------------\r\n");
        RN4871_SendString("No Valid Data");
        RN4871_SendString("\r\n------------------------------------------\r\n");
    }
}

/**
 * @brief This functions specifies actions to be taken if movement is detected.
 * @param None
 * @return None
 */
void setAlarm(void)
{
    printf("PIR : MOVEMENT DETECTED\r\n");
    
    LED0_ON();
    Timer0_Start();

    if (isBLEConnected == true)
    {
        RN4871_SendString("\r\n------------------------------------------\r\n");
        RN4871_SendString("Movement Detected");
        RN4871_SendString("\r\n------------------------------------------\r\n");
    }
}

/**
 * @brief User Interrupt handler for IOC for RN4871_UART_TX_IND pin
 *      Interrupt occurs when BLE module is about to send data over UART
 * @param None
 * @return None
 */
void RN4871_UART_TX_IND_IOC_UserInteruptHandler(void)
{
    isDatacomingFromBLE = true;
}

/**
 * @brief ADCC threshold interrupt callback function
 * @param None
 * @return None
 */
void ADCC_UserThresholdInterrupt(void)
{
    //check if threshold interrupt is occurred due to Accumulator overflow
    if (ADCC_HasAccumulatorOverflowed())
    {
        //Clear accumulator will clear accumulator overflow bit
        ADCC_ClearAccumulator();
    }
    else
    {
        isADCThresholdInterruptOccured = true;
    }
}

/**
 * @brief Timer 0 interrupt callback function
 * @param None
 * @return None
 */
void TMR0_UserInterruptHandler(void)
{
    Timer0_Stop();
    LED0_OFF();
}

/**
 * @brief Turns on LED0
 * @param None
 * @return None
 */
inline void LED0_ON(void)
{
    LED0_SetLow();
}

/**
 * @brief Turns off LED0
 * @param None
 * @return None
 */
inline void LED0_OFF(void)
{
    LED0_SetHigh();
}

