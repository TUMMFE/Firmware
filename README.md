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
### Software Architecture
#### Communication Protocol
##### Host to µC
##### µC to Host

### Bugs & Not Implemented Yet

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
