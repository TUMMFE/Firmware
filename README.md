# Firmware for Acceleration Sensor Project
The Acceleration Sensor Project is a experimental setup used for muscle training and as readout for the experimental verification of the neuronal models for magnetic peripherial stimulation.

The acceleration sensor will be placed on the thumb. The connection to the host is done using USB for data transmission and for power supply.

## Hardware
- ADXL355 3-axis MEMS sensor
- Board V1.0 with
  - Piezo Buzzer for acoustic stimulation used muscle training experiments. Subjects practice fastest possible thumb flexion movements by performing brief contractions of their left flexor pollicis brevis (FPB) muscle. Movements are externally paced by an acoustic stimulus at a certain frequency. Acoustic stimulus will be applied for several minutes, followed by a pause of again severel minutes. This blocks will be repeated several times.
  - LED error indicators
  - SPI connection to ADXL355 sensor
  - STM32F401 microcontroller
  - USB 2.0 Full Speed for data and power supply
  - SWD interface

## Project Details
### Bugs
no test - no bugs

### Roadmap
no test - no ideas

### Contributing
see GitHub commits

### Authors and acknowledgment
So far no external authors and no copied source code

## Project status
still under development - not tested on hardware

***
## Software Architecture

### How To Compile
Source can be build using KEIL µVision 5.38. Source uses KEIL RTX V2.0 as well as the STM32 USB drivers.

### Communication with the Host
#### Host --> Device
#### Device --> Host