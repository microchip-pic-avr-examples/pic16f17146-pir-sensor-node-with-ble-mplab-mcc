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


#include "mcc_generated_files/system/system.h"
#include "rn4871_click.h"
#include "pir_sensor.h"
#include "application.h"

volatile bool isDatacomingFromBLE = false; 
volatile bool isADCThresholdInterruptOccured = false;
volatile bool isBLEConnected = false; 

void ApplicationSetup(void)
{
    CPUDOZEbits.IDLEN = 0; //clear idle enable bit so that MCU will go to sleep mode when SLEEP instruction is executed
    
    RN4871_StopObservingIncomingData();
    PIR_StopObservingOutputChange();
    
    RN4871_UART_TX_IND_SetInterruptHandler(RN4871_UART_TX_IND_IOC_UserInteruptHandler);
    ADC1_SetADTIInterruptHandler(ADCC_UserThresholdInterrupt);
    Timer0.TimeoutCallbackRegister(TMR0_UserInterruptHandler);  
    
    printf("BLE : Setting up BLE Module..\r\n");
    RN4871_Setup();
    printf("BLE: BLE Setup Completed. Connect your Smartphone \r\n");
    RN4871_StartObservingIncomingData();
    
    printf("PIR: Setting up PIR sensor..\r\n");
    PIR_Setup(); 
    //ADRPT 64; 
    ADRPT = 0x40; // as there is a bug in the MCC generated ADC initialization code, we are setting this value here
    printf("PIR: PIR sensor setup completed\r\n");
    PIR_StartObservingOutputChange();
}

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
        printf("ADC Threshold Interrupt occurred\r\n");
        isADCThresholdInterruptOccured = false;      
        
        if(ADC1_HasErrorCrossedLowerThreshold())
        {
            //Movement detected    
            setAlarm();       
            printf("ADC Threshold Interrupt set to : ADERR > ADUTH\r\n");
            //Configure ADC to Interrupt if ADERR > ADUTH. 
            //Interrupt will occur if there is no movement.
            ADCON3bits.TMD = ADCTMD_ADERR_GREATHER_THAN_ADUTH; 
        }
        else if (ADC1_HasErrorCrossedUpperThreshold())
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

void RN4871_UART_TX_IND_IOC_UserInteruptHandler(void)
{
    isDatacomingFromBLE = true;
}

void ADCC_UserThresholdInterrupt(void)
{
    //check if threshold interrupt is occurred due to Accumulator overflow
    if (ADC1_HasAccumulatorOverflowed())
    {
        //Clear accumulator will clear accumulator overflow bit
        ADC1_ClearAccumulator();
    }
    else
    {
        isADCThresholdInterruptOccured = true;
        PIR_StopSampling();
    }
}


void TMR0_UserInterruptHandler(void)
{
    Timer0_Stop();
    LED0_OFF();
}

inline void LED0_ON(void)
{
    LED0_SetLow();
}

inline void LED0_OFF(void)
{
    LED0_SetHigh();
}

