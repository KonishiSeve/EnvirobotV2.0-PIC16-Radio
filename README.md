# Radio PIC16 firmware
This repository contains a MPLab project for the PIC16LF876A responsible of the radio interface on the head PCB of Envirobot V2.0.
## Content
### main.c
Implements a state machine waiting for incoming radio packets, translate the packet to the "framework" protocol and transmits it to the "head" STM32 of Envirobot V2.0 through UART.
The reponse from the STM32 is translated back to "legacy" protocol and transmitted back trough the radio.
This firmware also provides internal PIC16 register for configuration purposes.
### framework_protocol.c
Provides functions to encode and decode UART packets with content defined by the "framework" protocol created by Dorian Bignet for Envirobot V2.0
### radio_protocol.c
Provides functions to decode radio packets with content defined by the "legacy" protocol used with Envirobot V1.5
### mapping.c
Provides functions to manage the mappings in the EEPROM memory
### nrf905.c
Driver for the NRF905 radio IC connected to the PIC16 through SPI
### peripherHAL.c
Low level Hardware Abstraction Layer (HAL) for different peripherals of the PIC16LF876A. Only the functions that were needed for the Envirobot V2.0 project were implemented
### utilities.c
Provides diverse utility functions used during this project
