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
### Features
- USB CDC Device
- four different operation modes automatic acoustic stimulation mode, free running mode, external trigger mode and single shot mode with and without external trigger capability.

### Prerequisites & How To Compile

*Prerequisites*
- Keil MDK-ARM Professional, Version 5.38a
- Keil RTOS, RTX2

*How To Compile*
- ARM Compiler Version 6
- C99 


### Software Architecture
#### Description
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
- contains some macros to do some simple bit operations.
- defines the used microcontroller family
- defines the used microcontroller type
- defines the HSE clock value in Hz
- defines the event flags
	+ **FLAG_EXTI_DRDY:** This flag is set by the external data ready interrupt to signal that new data is available at the sensor
	+ **FLAG_EXTI_TRIGGER_IN:** This flag is set by the external trigger input interrupt to signal that a new data aquisition must be started
	+ **FLAG_SYSTEM_CONFIG_CHANGED:** This flag is set by the USB Rx thread when a configuration, state or mode change is requested by the USB host
	+ **FLAG_START_DAQ:** This flag is set by the sound system in automatic acoustic stimulation mode at each time, when a beep is started. This is used to signal the main thread that now data will be available soon.
- defines the communication protocol like the used Rx/Tx data frame length (in bytes), the command bytes and some min/max values to verfify data coming from the USB host 
- Ports and Pins for GPIO outputs and inputs, the SPI interface, the TIMER for the buzzer but not the pins for the USB interface.

###### main.c/main.h
- contains the main program entry point *int main(void)* which
	+ configures the clock of the microcontroller
	+ generates the main threas *app_main* and
	+ starts the RTOS scheduler
- contains the USB receive callback function *CDC_Receive_FS_Callback()* which
	+ is executed when some data from the USB host was received
	+ reads the data from the input FIFO
	+ decodes the data
	+ stores data in the *SystemConfiguration* struct
- contains the *HAL_GetTick* function which is used for the RTOS SysTick timer together with the global variable *os_time* which is declared in *main.c* but defined in the RTOS system
- contains the *Error_Handler* function which
	+ is called on every configuration error/system error
	+ disables are interrupts
	+ resets the microcontroller using *NVIC_SystemReset*
- contains some private functions
	+ *StandardConfiguration()*
		* sets the basic configuration which is sensor in standby,  output data rate 1000 Hz, no highpass filter, range +/- 8g, no offset
		* inits the sound.c
	+ *ChangeConfiguration()* which writes the configuration to the sensor 		
	+ *USBTransmitData()* generates the byte stream from the data and transmit it via USB
	+ *ChangeOperationMode()* attaches and detaches the external interrupt lines, dempending on the operation mode which can be the automatic acoustic stimulation mode, the external trigger mode, the single shot mode or the free running mode
		* **START:** set the sample counter and the event counter to 0, start the measurement, attach der data ready interrupt and in case of automatic acoustic stimulation start the sound protocol.
		* **STOP:** detach the data ready interrupt, resets the sample and event counter and in case of the automatic acousting mode stops the sound generation
		* **PAUSE:** detach the data ready interrupt and in case of automatic acousting stimulation mode pauses the sound generation. No counters will be resetted.
		* **RESUME:** attaches the data ready interrupt again and in case of automatic acousting stimulation mode resumes the sound generation. No counters will be resetted.
	+ *ChangeState* starts, pauses, resumes or stops the 
- contains the main thread *app_main()* which
	+ configures the GPIO pins (not the alternate function pins)
	+ calls the *StandardConfiguration()* function
	+ configures the interrupt pins of the ADXL355
	+ has a endless loop in which
		* checks if a **FLAG_SYSTEM_CONFIG_CHANGED** was set, in case this flag was set call either *ChangeConfiguration()*, *ChangeState* or *ChangeOperationMode()* depending on the nature of configuration request
		* checks the current operation mode
			- in automatic acoustic stimulation mode, wait for **FLAG_START_DAQ** and than wait for data ready flag **FLAG_EXTI_DRDY** collect the data and transmit the data via USB. Do this until the number of requested samples per stimulus is reached or a stop or pause request was received
			- in free running mode wait for **FLAG_EXTI_DRDY** collect the data and transmit the data via USB. Do this as long as no stop request was received.
			- in external trigger mode: not implemented yet
			- in single shot mode wait for **FLAG_EXTI_DRDY** collect the data and transmit the data via USB. Do this until the number of requested samples per stimulus is reached or a stop or pause request was received. The difference to the automatic mode is, that in this mode the execution will **not** wait for a **FLAG_START_DAQ**			

###### stm32f4xx_it.c/stm32f4xx_it.h
- contains the decleration of the global USB handle *hpcd_USB_OTG_FS*
- contains the global IRQ handler for the EXTI (the one from the SGLib is used) which set the event flags defined in *bsp.h* dependant on the GPIO pin
- contains the interrupt handler for the USB connection *OTG_FS_IRQHandler* which in turn will call the *CDC_Receive_FS_Callback()* function at the far end
- NMI interrupt handler
- Hard fault interrupt handler

###### sound.c/sound.h
- contains some functions to generate an acoustic stimulation pattern using two software timers
	+ *stimulus timer* which starts the PWM which generates the PWM when started and stops the PWM on each tick. This timer is used to limit the duration of one acoustic stimulus
	+ *block timer* which controls the pause between two adjacent acoustic stimuli
- contains the GPIO and timer configuration for the buzzer
- the timer is set to PWM mode with a frequency of 4 kHz with a duty cycle of 50%

###### adxl355.c/adxl355.h
- contains a driver for the ADXL355 accelerometer sensor
- configuration of the SPI interface (SPI interface is used in interrupt mode)
- interrupt handler for the SPI interface (general SPI IRQ handler, transfer complete handler, receive complete handler and error handler)
- functions to write to and read data from the sensor
- function to configure the sensor

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
- external trigger modes are not implemented yet.
- seperate ADXL355 driver from underlying hardware (SPI)
- seperate tasks and thread in the main.c file

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
