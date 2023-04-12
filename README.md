# Acceleration Sensor Project
The Acceleration Sensor Project which is used to measure the acceleration of the thumb after transcranial magnetic stimulation.

The project consists of the following parts:
* Hardware
* Firmware
* PC Software
* Experiments


### Literature
The experiment is based on

 [1] Walther et al, "Deafferentation of neighbouring motor cortex areas does not further enhance saturated practice-dependent plasticity in healthy adults", Clinical Neurophysiology 119 (2008) 886-891, [doi:10.1016/j.clinph.2007.12.006](https://www.sciencedirect.com/science/article/abs/pii/S1388245707009017)

 [2] Delvendahl et al, "The time course of motor cortex plasticity after spaced motor practice", Brain Stimulation 4 (2011) 156-64, [doi:10.1016/j.brs.2010.10.002](https://www.brainstimjrnl.com/article/S1935-861X(10)00161-0/fulltext)

## Hardware
### Status
### Features
- Microcontroller: STM32F401RET6
- Crystal: 8 MHz
- USB 2.0 Full Speed with USB 3.0 connector 
- Piezo Buzzer: PKLCS1212E4001-R1 with 4 kHz resonance frequency
- JTAG interface
- Power LED
- Status LED
- ADXL355 sensor Low Noise, Low Drift, Low Power,
3-Axis MEMS Accelerometers with 20-bit resolution on a seperate PCB
- Connection between µC board and ADXL board: 10-way, FPC connector, Würth Part Number: 687310124422
- External trigger input

### Hardware Architecture
#### ADXL355
The sensor is connected to the microcontroller via SPI. Interrupt and Data Ready Pins are connected to digital inputs.

##### Pinning
The following pins are used:

ADXL355|µC |Description|Type @ µC
-------|---|-----------|---------
CS     |PA1|Chip select|output
INT1   |PA4|interrupt 1|input
INT2   |PA2|interrupt 2|input
MOSI   |PA7|SPI        |alternate function
MISO   |PA6|SPI        |alternate function
SCLK   |PA5|SPI        |alternate function
DRDY   |PA0|Data ready |EXTI

##### Power supply of the ADXL355.
The internal LDO will be used, thus, both **Vsupply** and **Vddio** will be powered by the same power supply like the mircocontroller. Power supply voltage is 3.3 V.

#### External Trigger
The external trigger can be used to synchronize the data acquisition of the ADXL355 with an external device/stimulator.

The following pins are used:

µC |Description|Type @ µC
---|-----------|---------
PA3|trigger in |EXTI

#### LED
One LED is connected to the 3.3 V power supply, a second LED is connected to the microcontoller.

The following pins are used:

µC |Description|Type @ µC
---|-----------|---------
Pxx|LED        |Output


#### Piezo Buzzer
The buzzer is connected to a PWM output channel of a timer peripherial of the microcontroller.

µC |Description|Type @ µC
---|-----------|---------
PA8|PWM output |alternate function

### Bugs & Not Implemented Yet

## Firmware
### Status


### Features
### Prerequisites & How To Compile

*Prerequisites*
- Keil MDK-ARM Professional, Version 5.38a
- Keil RTOS, RTX2

*How To Compile*
- ARM Compiler Version 6
- C99 


### Software Architecture
#### Description

#### Modules
##### Main
The files are located in the **Source** directory.

File         |Content
-------------|-----------------------
|bsp.h       |board supply package
|main        |main programm entry
|stm32f4xx_it|interrupt handling
|sound       |control of the buzzer
|adxl355     |interface tp ADXL355 sensor


###### bsp.h
- Contains some macros to do some simple bit operations.
- defines the used microcontroller family
- defines the used microcontroller type
- defines the HSE clock value in Hz
- defines the event flags
	+ FLAG_EXTI_DRDY: This flag is set by the external data ready interrupt to signal that new data is available at the sensor
	+ FLAG_EXTI_TRIGGER_IN: This flag is set by the external trigger input interrupt to signal that a new data aquisition must be started
	+ FLAG_SYSTEM_CONFIG_CHANGED: This flag is set by the USB Rx thread when a configuration, state or mode change is requested by the USB host
	+ FLAG_START_DAQ: This flag is set by the sound system in automatic acoustic stimulation mode at each time, when a beep is started. This is used to signal the main thread that now data will be available soon.
- defines the communication protocol like the used Rx/Tx data frame length (in bytes), the command bytes and some min/max values to verfify data coming from the USB host 
- Ports and Pins for GPIO outputs and inputs, the SPI interface, the TIMER for the buzzer but not the pins for the USB interface.

###### main.c/main.h

###### stm32f4xx_it.c/stm32f4xx_it.h

###### sound.c/sound.h

###### adxl355.c/adxl355.h

##### USB Device
The files are located in the **Middleware** directory.
The files contains the USB drivers from STM.

File        |Content
------------|-----------------------
|usbd_conf  |GPIO configuration of USB peripherial
|usbd_device|USB Device implementation
|usbd_cdc_if|USB VCP (CDC) implementation - *CDC_Receive_FS()* changed to call user *CDC_Receive_FS_Callback()* function (in main.c)
|usbd_desc  |USB Device Descriptors - PID/VID/Manufacturer info

##### Middleware
The files are located in the **Middleware** directory.
The files contains the USB drivers from STM as well as a library for a lightweight ring buffer.

File        |Content
------------|-----------------------
|lwrb       |Lightweight ring buffer
|usbd_cdc   |USB CDC Device class
|usbd_core  |USB Core
|usbd_ctlreq|USB Control Requests
|usbd_ioreq |USB IO Endpoints

##### SG Lib
The files are located in the **SGLib** directory.
Contains library function for GPIO, RCC and I2C. These files can be used for the following microcontrollers:
- STM32F0xx
- STM32F401xx
- STM32F405xx
- STM32F407xx
- STM32F410Cx
- STM32F410Rx
- STM32F410Tx
- STM32F415xx
- STM32F417xx
- STM32F427xx
- STM32F429xx
- STM32F437xx
- STM32F439xx
- STM32F446xx
- STM32F469xx
- STM32F479xx
- STM32F7xx

These functions can be used generally. There is a own GIT project for this library.

##### CMSIS
Keil RTX Configuration files. In **RTX_Config.h** the RTOS can be configured.
There are no special changes for this project beside the standard settings.
 
##### Device
Contains the HAL Library from STM.
#### Communication Protocol
##### Host to µC
##### µC to Host

### Bugs & Not Implemented Yet
- external trigger mode is not implemented yet.
-

## PC Software
### Status
### Features
### Prerequisites & How To Compile
### Software Architecture
### Bugs & Not Implemented Yet
## Experiments
Subjects practice fastest possible thumb flexion movements by performing brief contractions of their left flexor pollicis brevis (FPB) muscle. Movements are externally paced by an acoustic stimulus at a certain frequency. Acoustic stimulus will be applied for several minutes, followed by a pause of again severel minutes. This blocks will be repeated several times.

### Authors & Contribution

What         |Who
-------------|---------------
Firmware     |Bernhard Gleich
Software     |Bernhard Gleich
Hardware     |Bojan Sandurkov
Data analysis|Jonathan Rapp
Experiments  |----
