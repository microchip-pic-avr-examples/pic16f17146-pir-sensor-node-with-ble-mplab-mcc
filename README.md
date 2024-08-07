
[![MCHP](images/microchip.png)](https://www.microchip.com)

# PIR Sensor Node With BLE Using PIC16F17146

## Overview
This example can be divided into two parts – Passive Infrared (PIR) sensor  interface and Bluetooth® Low Energy (BLE) communication.
Both of these parts are independent to each other meaning any choice of sensor can be used in the place of PIR sensor. This document first covers the overview of the PIR sensor interface followed by BLE communication.

![sensor-node](images/sensor-node.png)

## Related Documentation
-	[PIC16F17146 Product Page](https://www.microchip.com/en-us/product/PIC16F17146?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_PIC16F171xx&utm_content=pic16f17146-pir-sensor-node-with-ble-mplab-mcc-github&utm_bu=MCU08)
-	[PIC16F17146 Data Sheet](https://www.microchip.com/DS40002343)

## Software Used
- [MPLAB® X IDE 6.20 ](https://www.microchip.com/en-us/tools-resources/develop/mplab-x-ide?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_PIC16F171xx&utm_content=pic16f17146-pir-sensor-node-with-ble-mplab-mcc-github&utm_bu=MCU08) or newer
- [Microchip XC8 Compiler 2.46](https://www.microchip.com/en-us/tools-resources/develop/mplab-xc-compilers?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_PIC16F171xx&utm_content=pic16f17146-pir-sensor-node-with-ble-mplab-mcc-github&utm_bu=MCU08) or newer
- [MPLAB® Code Configurator (MCC) 5.5.1](https://www.microchip.com/en-us/tools-resources/configure/mplab-code-configurator?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_PIC16F171xx&utm_content=pic16f17146-pir-sensor-node-with-ble-mplab-mcc-github&utm_bu=MCU08) or newer
-	[Microchip PIC16F1xxxx Series Device Support (DFP) 1.25.389](https://packs.download.microchip.com/) or newer
- [Microchip Bluetooth Data iOS® app 3.0](https://apps.apple.com/us/app/microchip-bluetooth-data/id1319166097) or newer
- [Microchip Bluetooth Data Android™ app 6.1 ](https://play.google.com/store/apps/details?id=com.microchip.bluetooth.data) or newer
- [RN4871 Firmware 1.41 ](https://www.microchip.com/product/RN4871) or newer

## Hardware Used
-	[PIC16F17146 Curiosity Nano Board](https://www.microchip.com/en-us/development-tool/EV72J15A?utm_source=GitHub&utm_medium=TextLink&utm_campaign=MCU8_PIC16F171xx&utm_content=pic16f17146-pir-sensor-node-with-ble-mplab-mcc-github&utm_bu=MCU08)
-	[Curiosity Nano Base Board](https://www.microchip.com/development-tool/AC164162)
-	[RN4871 Click Board](https://www.mikroe.com/rn4871-click)
- [PIR Click Board](https://www.mikroe.com/pir-click)

## PIR Sensor Interface
This example aims to simplify the PIR sensor interface to a Microcontroller (MCU) using intelligent analog peripherals such as Operational Amplifier (OPA), Analog-to-Digital converter with computation (ADCC), Digital-to-Analog converter (DAC), Fixed Voltage Reference (FVR), and core independent peripherals (CIP) such as the Timer of PIC16F17146 MCU. The use of intelligent and core independent peripherals makes it possible to interface a PIR sensor with minimal external components and minimal CPU intervention. Minimal CPU intervention enables the opportunity of operating the MCU in sleep mode for most of the time.

The output signal of the PIR sensor is amplified and digitally filtered to determine if the movement has occurred. The techniques used in this example can be also applied to other sensor interfaces where amplification of row sensor output and noise removal is required.

## PIR Sensor Overview
A PIR sensor detects changes in the amount of infrared radiation “seen” by the sensor elements, which varies depending on the temperature and surface characteristics of the object in front of the sensor. When a person passes between the sensor and the background, the sensor detects the change from ambient temperature to body temperature, and back again. The sensor converts the resulting change in the infrared radiation into a change in the output voltage.

### Analysis of PIR Sensor Output
The figure below shows a plot of the raw output of the PIR sensor captured using the oscilloscope. A hand is placed over the sensor and held there for a short time and then removed.

![pir-output](images/pir-output.png)

Below are the observations of the captured data:
- Voltage is around 800 mV during the normal condition (no movement)
- Voltage goes up to 820 mV when there is movement in front of the sensor
- Voltage goes down to 780 mV before settling around 800 mV again
- There is high frequency noise in the PIR signal as the signal is not smooth

**Note:** There may be variations in output voltage levels from sensor to sensor but the working principle of the sensor remains the same.  

### Implementation Details
The following measures need to be taken to detect the change in the PIR sensor output:
1. The sensor output needs to be amplified before being sampled by the ADC, as the change in output signal levels from the PIR sensor is very low.
2. The DC offset voltage present in the PIR signal at normal conditions needs to be removed before amplification.
3. High-frequency noise needs to be removed to avoid false detection when the noise level is high.

![block-diagram-pir](images/block-diagram-pir.png)

The first two measures stated above are addressed by using OPA and DAC peripherals. OPA is used in inverting amplifier mode. DAC output is applied at the non-inverting terminal of OPA to remove DC offset voltage. OPA uses an internal resistor ladder for providing the gain required for amplification. The FVR module provides stable voltage reference irrespective of supply voltage to the DAC. To remove unwanted high-frequency noise from the PIR signal, ADCC is used in the Low Pass Filter (LPF) mode. ADCC is triggered periodically using Timer 2. ADCC’s threshold interrupts feature is used to generate an interrupt whenever there is a change in output.

### Amplification and DC Bias Removal

As observed previously, the PIR signal is at 800 mV in normal conditions. There is a change of 20 mV to 100 mV when movement is detected. To remove the DC bias of 800 mV and to amplify only change, OPA can be used in difference amplifier mode as shown in the below figure.

![opa-circuit1](images/opa-circuit1.png)

For the above circuit, output can be given as:

`output=(-R2/R1)(PIR voltage - DC bias voltage)`

As the OPA uses a single supply, extra DC bias voltage needs to be applied at the non-inverting terminal to shift the signal up. Voltage is shifted to V<sub>DD</sub> / 2  in order to cover the maximum output range.

Extra DC bias voltage required can be calculated as:

`(VDD ⁄ 2)/(R2 ⁄ R1)`

Thus, total DC voltage needs to be applied is:

`0.8 V + (VDD ⁄ 2)/(R2 ⁄R1)`

![opa-circuit2](images/opa-circuit2.png)

Thus,

`Output =  -R2/R1 * (PIR voltage - 0.8 - (VDD⁄ 2)/(R2 ⁄ R1))`

`Output=  VDD/2 - (PIR voltage - 0.8) * R2/R1`

From the above equation, it is noticeable that, 0.8 V (PIR offset voltage) is removed from the PIR signal and then the signal is amplified by the gain set by R2 and R1. The resulting signal deviates around V<sub>DD</sub> / 2  covering the maximum output range.

The OPA module of PIC16F17146 has an internal resistor ladder which can be used at the inverting terminal of OPA for this application. At the non-inverting terminal, DC voltage can be applied directly using the DAC module without any resistors. The DC voltage needs to be set by considering the voltage divider formed by the resistor (R1 and R2).

For this application, R1 and R2 are set as 20k and 300k, respectively, which leads to a gain of 15. Thus, the voltage required to apply at the non-inverting terminal without any resistors is:

`(0.8 + (VDD ⁄ 2)/(R2 ⁄ R1)) * R2 / (R2+R1) = (0.8 + 1.65 / 15)* 300 / 320 = 0.853V`


DAC2 is used to generate 853 mV and its output is connected to OPA’s non-inverting terminal internally. Finally, the circuit will look as shown below.

![opa-circuit3](images/opa-circuit3.png)

The figure below shows the output of OPA.

![mindi-output](images/mindi-output.png)

The change in the PIR signal is amplified. The OPA output is inverted as the PIR signal is connected to the inverting terminal of OPA. When there is no movement, the output is around 1.65 V ( V<sub>DD</sub> / 2 ).

**Note:** The simulation above is generated using [MPLAB® Mindi™ Analog Simulator](https://www.microchip.com/tools-resources/develop/analog-development-tool-ecosystem/mplab-mindi-analog-simulator). Mindi model for PIC16F17146's OPA is available [here](https://github.com/microchip-pic-avr-examples?q=pic16f17146-opamp-mindi).

### Noise Cancellation
The PIR signal has high-frequency noise. The ADCC’s LPF is used to remove unwanted high frequency components of the signal. The LPF is a single-pole, unity gain digital filter and is one of the computation features of the ADCC. The ADCRS value determines the -3 dB roll-off frequency. The table below shows the radian values at the -3 dB roll-off frequency based on ADCRS values.

![adcc-lpf](images/adcc-lpf.png)

The radian values listed in the table above are defined by the ADCC hardware. These values are used to calculate the -3 dB roll-off point in terms of frequency. The following equation can be used to determine the -3 dB point.

`Frequency @ -3dB = (Radians @ - 3dB) / 2πT`

Where:<br>
Radians @ -3 dB = the value from the table above based on the ADCRS value.<br>
T = total sampling time. It is the measured time between samples.

For detailed operation of ADCC LPF, refer to [AN2479](https://www.microchip.com/DS00002749).

For this example, ADCRS is set as 6.

To set the cutoff frequency to 3 Hz, T should be around 840 us.

**Note:**
1. Cutoff frequency of 3 Hz is set to match RC low pass filter cutoff frequency present on the click board.
2. Timer 2 is used to trigger the ADCC in every 840 us.

The figure below shows the output of ADCC. There is a change in output values whenever the hand is moved across the PIR sensor.

![pir-output-uart](images/pir-output-uart.png)

**Note:** The ADCC data is sent to the PC using EUSART and plotted by using the Data Stream protocol in the [Data Visualizer](https://www.microchip.com/en-us/tools-resources/debug/mplab-data-visualizer). The figure is for reference and firmware does not include data streaming instructions. For detailed information on Data Visualizer, refer to the [Data Visualizer User's Guide](https://www.microchip.com/DS50003001).

### Sleep Mode Operation and ADCC Threshold Interrupt
In this example, the ADCC threshold interrupt is used to generate an interrupt whenever there is a change in ADCC output. The threshold comparison is performed on the final filtered value. When there is a change in filtered output by the amount set by the ADCC threshold value, it indicates that movement has been detected. Various operations can be performed on ADCC output to generate a threshold interrupt. Please refer to the device data sheet for details.

 Furthermore, when the ADCC is configured, the ADCC generates an interrupt when ADCC filtered value goes below some predefined value (lower threshold) and thus indicating movement is detected. The threshold value ultimately determines the sensitivity of the sensor. The higher the threshold value, the lower the sensitivity and vice versa.

To perform movement detection in sleep mode, ADCC and TMR2 (which is used to trigger ADCC), should run even if the MCU is in sleep mode.  ADCC can run in sleep mode by using the ADCRC clock. TMR2 can also be made to operate in sleep by using LFINTOS as its clock source.  

After movement is detected, the MCU wakes up due to a lower threshold interrupt. Once woken up, the MCU disables the lower threshold interrupt and enables the upper threshold. Then the MCU goes to sleep again. An upper threshold limit is set to generate an interrupt when PIR output stabilizes at a normal value (PIR output when there is no movement). Thus, when there is no movement, the MCU wakes up due to an upper threshold interrupt. Once woke up, the MCU disables the upper threshold interrupt and enables a lower threshold.

When movement is detected, the MCU turns on the LED for five seconds. The MCU goes to sleep immediately after turning on the LED. TMR0 generates an interrupt after five seconds to turn off the LED. This period can be changed, per application, as needed.  

## RN4871 BLE Module Interface
RN4871 communicates with the host controller using ASCII commands over the  Universal Asynchronous Receiver/Transmitter (UART) interface. RN4871 acts as a peripheral device and the smartphone acts as a central device throughout this application.

In this example, whenever movement is detected by the PIR sensor, a message is sent to the smartphone. Also, PIR sensitivity can be updated by sending data from the smartphone to the BLE module.

Previously, the MCU operates most of the time in sleep mode. The BLE module’s UART_TX_IND pin is used to wake the MCU up to receive an incoming message from the BLE module. UART_TX_IND pin goes low before RN4871 sends data over UART. The MCU is configured to generate an interrupt by waking up the MCU when the UART_TX_IND line goes low.  

![block-diagram-ble](images/block-diagram-ble.png)

## BLE Communication

### Transparent UART
This example uses a private Generic Attribute Profile (GATT) service named “Transparent UART” introduced by the RN4871 software stack. Transparent UART transfers serial data from its UART over a Bluetooth Low Energy connection, providing an end-to-end data pipe to another Bluetooth device.

Refer to [RN4870/71 Bluetooth Low Energy Module User’s Guide](https://www.microchip.com/DS50002466) for details.

### Smartphone App
This example uses the Microchip Bluetooth Data (MBD) app for the demonstration. BLE UART feature on the app scans and connects the BLE device (RN4871 module). The app displays data received from the device on-screen and transfers text typed in the app to a BLE device.

Visit [Bluetooth® Low Energy webpage](https://www.microchip.com/en-us/products/wireless-connectivity/bluetooth-low-energy) for app link and source code.

**Note:** Some of the RN4871 factory settings have been changed before interfacing with PIC16F17146. Using the S-command, the device name has been changed to Sensor_Node. Also, the UART communication settings have been configured to 9600,8,N,1.

## Hardware Setup
The image below shows the hardware setup. PIC16F17146 Curiosity Nano board is used as a development platform in this example. Curiosity Nano base board is used for connecting click boards to the Nano board.

![setup](images/setup.jpeg)

#### Hardware Modifications on PIR Click:
The PIR Click board is used for demonstration purposes. The Click board needs to be modified before it can be used for this example. The original Click board has an external amplification and filter stage implemented using OPA and passive components which is shown in the figure below.

![pir-click-modification](images/pir-click-modification.png)

For this example, no external components are required (except a 47k resistor at the output of the sensor which is recommended by the sensor data sheet). The R11 resistor from the Click board is removed and PIR output (VOUT) is directly connected (green cable in the image present in [Hardware Setup](https://github.com/microchip-pic-avr-examples/pic16f17146-pir-sensor-node-with-ble-mplab-mcc#Hardware-Setup)) to the one of the NC pins (RST) of the mikroBUS<sup>TM</sup> header to bypass the amplification and filter stage.

#### Hardware Modifications on RN4871 Click:
The RN4871 Click does not provide the connection for RN4871’s UART_TX_IND pin on the mikroBUS header. Thus, a wire is explicitly soldered to connect the UART_TX_IND pin (pin 15 – P2_7) and one of the NC pins (SCK) of the mikroBUS header.

![rn4871-click-modification](images/rn4871-click-modification.png)

**Note:** Mode switch on RN4871 Click board should be set to <span style="font-family: Courier New;">0 </span> in order to put RN4871 in application mode.

## Hardware Connection
![connection](images/connection.png)

## Pin Connection Table

| Pin | Signal Description                    
| --- | -------------------                
| RC3	| OPA Negative Channel<sup>2</sup>
| RA5 | PIR output<sup>2,3</sup>
| RC1	| LED
| RC4	| EUSART2 RX<sup>1</sup>
| RC5	| EUSART2 TX<sup>1</sup>
| RC6	| RN4871 UART_TX_IND
| RA4	| RN4871 RST_N<sup>1</sup>
| RB5	| EUSART1 RX
| RB7	| EUSART1 TX

<sup>1</sup> Connection from the PIC16F17146 Curiosity Nano to Click slot is not available for these pins. Jumper cables are used to make the required connections. For RN4871 TX pin, short **RX2** and **MISO** of Curiosity Nano Base Board. For RN4871 RX pin, short **TX2**, and **MOSI**. For RN4871 RST_N pin, short **RST2** and **INT1**.

<sup>2</sup> OPA negative channel is not available for pin RA5. A jumper cable is used to connect PIR output (**RA5**) to OPA negative channel (**RC3**).

<sup>3</sup>Pin RA5 should be in tri-state to avoid any interference with PIR output.

## Demo Operation
1. When the device is powered on, the LED blinks for five seconds before turning off. The five seconds is the warm-up time set for this example. This is a one-time process. There should not be any movement around the sensor during this time.
2. Whenever movement is detected, the PIR sensor output goes below the threshold value. The LED turns on to indicate the movement. It stays on for five seconds before turning off. This process repeats whenever movement is detected. Also, a message is sent to the smartphone if a BLE connection is available.

![movement-detected](images/movement-detected.gif)
![movement-detected](images/movement-detected.png)

3. Whenever there is no movement, the LED remains in the off state.
4. The sensitivity of the PIR sensor can be updated by sending sensitivity values (0 to 10) from the smartphone app. The MCU acknowledges if it receives updated sensitivity by sending messages back to the smartphone app.

![sensitivity-update](images/sensitivity-update.gif)

#### Steps to Connect RN4871 BLE Module to Smartphone
1. Install and open the Microchip Bluetooth Data app.
2. Click on BLE UART.
3. Click on BM70.
4. Click on Scan. If a pop-up message appears to turn on Bluetooth, click Yes and then click Scan again.
5. Click the Scan tab to list the devices.
6. Wait four to five seconds, and then click Cancel to cancel the scan. After canceling the scan, the Bluetooth module name (Sensor_Node), should be listed on the screen.<br>
**Note:** If the name is not listed, go to Step 4 and repeat the scan.<br>
7. Click on the BLE Module name.
8. Wait for the connection. The LED on the RN4871 Click board will blink twice, continuously, when the connection is established.
9. Click on Transfer data to open the UART stream. Now, the data sent by the RN4871 can be observed and data can be sent to the RN4871.

![connection-steps](images/connection-steps.gif)

## Peripheral Configuration Using MCC
This section explains how to configure the peripherals using MPLAB X IDE with MCC plug-in to recreate the project.

Refer the [Software Used](https://github.com/microchip-pic-avr-examples/pic16f17146-pir-sensor-node-with-ble-mplab-mcc#software-used) section to install required tools to recreate the project.

Additional Links: [MCC Melody Technical Reference](https://onlinedocs.microchip.com/v2/keyword-lookup?keyword=MCC.MELODY.INTRODUCTION&redirect=true)


|    Module  |    Configuration                                                                                                                                                                                                                                                                                                                                                       |    Usage                                                                   |
|--------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------|
|    Clock Control           |    *Clock Settings*<br>Clock Source – HFINTOSC<br>   HF Internal Clock – 1 MHz<br>   Clock Divider – 1                                                                   |    System clock   
|    OPA                    |   *Hardware Settings*<br> Enable Op Amp<br>   Op Amp   Configuration – Inverting Programmable Gain Amplifier<br><br>*Channel Selection*<br>Positive Channel – DAC2_OUT<br>Negative Channel – GSEL<br>Negative Source Selection – OPA1IN1-  <br><br> *Programmable Gain and Feedback Selection* <br>Internal Resistor Ladder selection – R2/R1 = 15        |    Amplifies PIR output      |
|    DAC2                    |  *Hardware Settings*<br>  Enable DAC<br>   DAC Positive reference selection – FVR<br>   DAC Negative reference selection – VSS  |    Connects to OPA non-inverting terminal. Provides bias voltage.                                   |    
|    FVR                    |   *Hardware Settings*<br> Enable FVR<br>   FVR_buffer 1 Gain – off<br>   FVR_buffer 2 Gain – 2x       |    Provides reference voltage to DAC                                           |                                 
|    ADCC                    |   *Software Settings*<br> TMR Dependecy Selector - TMR2<br><br>*Hardware Settings*<br>Enable ADCC<br>   Input   Configuration- Single Ended Mode<br> Operating   Mode – Low Pass Filter Mode<br>Positive   Reference – VDD<br>Auto-conversion Trigger – TMR2<br>Result   Alignment – Right justified<br>Acquisition Count - 3<br><br> *Computation Settings*<br> Error Calculation Mode – Filtered value vs setpoint  <br> Threshold setpoint – 0 <br> Upper Threshold - 0<br>Threshold Interrupt Mode – ADERR < ADLTH <br> Repeat – 64 <br> Accumulator Right Shift – 6 <br><br> *ADC Clock settings* <br> Clock Source – ADCRC  <br><br> *Interrupt Settings*<br>ADTI Interrupt Enable|    Monitors OPA output                                          |
|    TMR2                    |   *Software Settings*<br> Dependency Selector – TMR2<br><br> *Hardware Settings*<br>Disable Timer<br>Control Mode – Roll over pulse<br>Start/Reset Option – Software control<br><br>*Timer Clock*<br> Clock Source – LFINTOSC <br> Prescaler – 1:1 <br> Postscaler – 1:1 <br><br>*Timer Period*<br>  Timer Period – 840 us         |    Triggers ADCC                                                      |
|   TMR0  |   *Hardware Settings*<br> Disable Timer<br>   Clock Prescaler  – 1:1024<br> Clock Source – LFINTOSC<br>Disable   Synchronisation<br>        Requested   Period – 5s<br><br> *Interrupt Settings*<br>  Enable TMR Interrupt|    Provides time to turn off the LED                                  |
|  EUSART1  |  *UART1 Driver*<br>Requested Baudrate –   9600 <br> UART PLIB Selector – EUSART1<br>Enable Redirect   STDIO to EUSART <br><br> *EUSART1 PLIB* <br>Enable Receive<br>  Enable Transmit<br> Enable Serial   Port                                                                                                                                                                                              |    Sends data to   PC terminal                                              |
|    EUSART2                 |    *UART2 Driver*<br>Requested Baudrate –   9600 <br> UART PLIB Selector – EUSART2 <br> Enable Interrupt Driven<br> Enable Redirect   STDIO to EUSART  <br><br> *EUSART2 PLIB:* <br>     Enable   Receive<br>  Enable Transmit<br>  *Interrupt Settings* <br> Enable RCI interrupt                                                                                                                                                                                            |    Communicates with BLE module                                              |
| Pins |  *Pins - Custom Name*<br> RC3 - RC3_IN1<br> RC1 - LED0<br> RC6 - RN4871_UART<br> RA4 - RN4871_RST_N <br><br> *Pin Grid View* <br> *EUSART1:*<br> RX1 - RB5<br> TX1 - RB7 <br>*OPA:*<br>OPA1INx<sup>-</sup> - RC3 <br> OPA1OUT - RC2 <br>*ADCC:*<br> ANPx - RC2 <br>*EUSART2:*<br> RX2 - RC4 <br> TX2 - RC5<br> *PINS:*<br> Input - RC6 <br> Output - RA4, RC1| Pin Configuration |

**Note:** The on-board debugger present on the Curiosity Nano board has a virtual serial port (CDC) that is connected to a UART on the PIC16F17146 and provides an easy way to communicate with the target application through terminal software. Refer to the Curiosity Nano User Guide for more details.

## Summary
This code example demonstrated interfacing of a passive infrared (PIR) sensor with BLE connectivity using various peripherals of PIC16F17146 microcontroller.
