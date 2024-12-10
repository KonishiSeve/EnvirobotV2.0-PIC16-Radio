/*
 * File:   main.c
 * Author: Séverin Konishi
 *
 * Created on 15. november 2024, 16:23
 */
// === CONFIGURATION BITS === //
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

// === INCLUDES === //
#include <xc.h>
#include "nrf905.h"
#include "utilities.h"
#include "peripherHAL.h"
#include "framework_protocol.h"
#include "radio_protocol.h"
#include "mapping.h"

// === DEFINES === //
//version number stored in register 0x3E0
#define VERSION         0x49

//where the PIC local registers start
#define PIC_REG_BASE        0x3E0
#define REG_PIC_VERSION     0x3E0
#define REG_PIC_CHANNEL     0x3E1

//radio channel
#define EEPROM_ADDR_CHANNEL     0x00
//number of mappings registered for indirect register access
#define EEPROM_ADDR_MAP_NB      0x01
//where the mappings storage starts (4 bytes per mapping)
#define EEPROM_ADDR_MAP_BASE    0x10

//maximum number of indirect mappings supported
#define INDIRECT_MAPPINGS_MAX   0x2F

#define UART_TIMEOUT            10000


// === Global Variables === //
// communication with the STM32 through UART
uint8_t uart_counter = 0;
uint8_t uart_buffer_rx[32];
uint8_t uart_buffer_rx_len = 0;
//set to 1 when UART framework packet received
//buffer will not be overwritten as long as it is not set to 0 by the user
uint8_t uart_flag_rx = 0;

// communication with the client through the NRF905 (with SPI)
uint8_t radio_buffer_rx[32];
//set to 1 when Radio packet received
//buffer will not be overwritten as long as it is not set to 0 by the user
uint8_t radio_flag_rx = 0;

//indirect address control registers
uint8_t reg_map_status_read = 0;
uint8_t reg_map_status_write = 0;
uint8_t reg_map_index = 0;

uint16_t reg_map_addr_radio = 0;
uint16_t reg_map_addr_frame = 0;
uint16_t reg_map_length = 0;

//Mapping control registers
#define REG_MAP_STATUS      0x3F0
#define REG_MAP_INDEX       0x3F1
#define REG_MAP_ADDR_RADIO  0x3F2
#define REG_MAP_ADDR_FRAME  0x3F3
#define REG_MAP_LENGTH      0x3F4

#define MAP_ERROR_ACCESS    0xEA
#define MAP_ERROR_FULL      0xEB
#define MAP_ERROR_INDEX     0xEC

//errors
#define ERROR_OK        0x00
#define ERROR_GENERIC   0x01
#define ERROR_SETUP     0x02
#define ERROR_REG_PIC   0x03
#define ERROR_REG_MAP   0x05
#define ERROR_STM32_TIMEOUT 0x05
#define ERROR_STM32_CHECKSUM 0x06
#define ERROR_STM32_RESPONSE 0x07
uint8_t error_state = ERROR_OK;

/* [DEBUG]
uint8_t debug[64] = {0};
uint8_t debug_counter = 0;
*/

// === Main ISR === //
void __interrupt() main_isr(void) {
    // === Interrupt from UART RX === //
    if(PIR1bits.RCIF) {
        //clear the interrupt flag and read the UART byte
        uint8_t uart_value = RCREG;
        //state machine to decode the UART framework packet (find start bytes + payload length)
        if(uart_counter==0 && uart_value==0xC3 && uart_flag_rx==0) {
            uart_buffer_rx[uart_counter++] = uart_value;
        }
        else if(uart_counter>=1) {
            //look for second start byte or abort reception of this packet
            if(uart_counter==1 && uart_value!=0x3C) {
                uart_counter = 0;
                return;
            }
            //look for payload length
            else if(uart_counter==3) {
                uart_buffer_rx_len = uart_value + 6;   //index at which the packet will stop: Start bytes(2) + Source (1) + Length(1) + Checksum(1) + End(1)
            }
            //last byte of packet (according to payload length)
            else if(uart_counter >= uart_buffer_rx_len-1 && uart_counter > 3) {
                uart_flag_rx = 1;
                uart_buffer_rx[uart_counter] = uart_value;
                uart_counter = 0;
                return;
            }
            uart_buffer_rx[uart_counter++] = uart_value;
        }
    }
    // === Interrupt on the DR pin of the nrf905 === //
    else if(INTCONbits.INTF) {
        INTCONbits.INTF = 0;    //clear the pin interrupt flag
        //only write to buffer if the main program is done processing the last packet
        if(!radio_flag_rx) {
            nrf905_receive(radio_buffer_rx);
            radio_flag_rx = 1;
        }
    }
}
// ===== PIC registers ===== //
//modifies the register_operation structure to send a response to the radio
void pic_register_read(reg_op* register_operation) {
    switch(register_operation->address) {
        case REG_PIC_VERSION: //Version register (992 in decimal)
            register_operation->value[0] = VERSION;
            register_operation->size = 1;
            return;
        case REG_PIC_CHANNEL: //Channel register (993 in decimal)
            register_operation->value[0] = eeprom_read(EEPROM_ADDR_CHANNEL);
            register_operation->size = 1;
            return;
        // == Mapping Registers == //
        case REG_MAP_STATUS: //mapping status register (994 in decimal)
            if(reg_map_status_read >= 0xE0) {
                //return the error code once if one happened
                register_operation->value[0] = reg_map_status_read;
                reg_map_status_read = 0;
            }
            else {
                reg_map_status_read = eeprom_read(EEPROM_ADDR_MAP_NB);
                register_operation->value[0] = reg_map_status_read;
            }
            register_operation->size = 1;
            return;
        case REG_MAP_INDEX:
            register_operation->value[0] = reg_map_index;
            register_operation->size = 1;
            return;
        case REG_MAP_ADDR_RADIO: //mapping radio address register (995 in decimal)
            register_operation->value[0] = reg_map_addr_radio&0xFF;
            register_operation->value[1] = reg_map_addr_radio>>8;
            register_operation->size = 2;
            return;
        case REG_MAP_ADDR_FRAME: //mapping framework address register (996 in decimal)
            register_operation->value[0] = reg_map_addr_frame&0xFF;
            register_operation->value[1] = reg_map_addr_frame>>8;
            register_operation->size = 2;
            return;
        case REG_MAP_LENGTH: //mapping length register (997 in decimal)
            register_operation->value[0] = reg_map_length&0xFF;
            register_operation->value[1] = reg_map_length>>8;
            register_operation->size = 2;
            return;
    }
}

void pic_register_write(reg_op* register_operation) {
    uint8_t mapping_count;
    switch(register_operation->address) {
        case 0x3E1: //Channel register (993 in decimal)
            eeprom_write(EEPROM_ADDR_CHANNEL, register_operation->value[0]);
            nrf905_set_channel(register_operation->value[0]);
            return;
        
        // === MAPPING STATUS REGISTER === //
        case REG_MAP_STATUS: //mapping status register (994 in decimal)
            mapping_count = eeprom_read(EEPROM_ADDR_MAP_NB);
            //read an existing mapping
            if(register_operation->value[0] < INDIRECT_MAPPINGS_MAX) {
                if(register_operation->value[0] >= mapping_count) {
                    reg_map_status_read = MAP_ERROR_INDEX;
                    return;
                }
                mapping_get(register_operation->value[0], &reg_map_addr_radio, &reg_map_addr_frame, &reg_map_length);
                reg_map_index = register_operation->value[0];
            }
            //create a new mapping or modify an existing one
            else if(register_operation->value[0] == 0xAA && reg_map_length > 0) {
                //modify existing mapping
                if(reg_map_index < mapping_count) {
                    mapping_set(reg_map_index, reg_map_addr_radio, reg_map_addr_frame, reg_map_length);
                }
                //no mapping at this index
                else if(reg_map_index != 0xFF) {
                    reg_map_status_read = MAP_ERROR_INDEX;
                    return;
                }
                //new mapping but no space left in eeprom
                else if(mapping_count+1 >= INDIRECT_MAPPINGS_MAX) {
                    reg_map_status_read = MAP_ERROR_FULL;
                    return;
                }
                else {
                    reg_map_status_read = mapping_new(reg_map_addr_radio, reg_map_addr_frame, reg_map_length);
                }
            }
            //reset all the mappings
            else if(register_operation->value[0] == 0xAC && reg_map_status_write == 0xAB) {
                eeprom_write(EEPROM_ADDR_MAP_NB, 0);
                reg_map_status_read = 0;
            }
            reg_map_status_write = register_operation->value[0];
            return;
        
        case REG_MAP_INDEX:
            reg_map_index = register_operation->value[0];
            return;
            
        case REG_MAP_ADDR_RADIO: //mapping radio address register (995 in decimal)
            reg_map_addr_radio = ((register_operation->value[1]&0b00000011)<<8) | register_operation->value[0];
            return;
            
        case REG_MAP_ADDR_FRAME: //mapping framework address register (996 in decimal)
            reg_map_addr_frame = ((register_operation->value[1]&0b000111111)<<8) | register_operation->value[0];
            return;
            
        case REG_MAP_LENGTH: //mapping length register (997 in decimal)
            reg_map_length = ((register_operation->value[1]&0b00000001)<<8) | register_operation->value[0];
            return;
    }
}


// ===== Indirect registers ===== //
//modifies the register operation structure (coming from the radio) to send it to the STM32
uint8_t indirect_register_access(reg_op* register_operation) {
    uint8_t mappings_nb = eeprom_read(EEPROM_ADDR_MAP_NB);
    uint8_t eeprom_readings[4];
    for(uint8_t i=0;i<mappings_nb;i++) {
        //read a mapping from the EEPROM
        eeprom_read_buffer(eeprom_readings, EEPROM_ADDR_MAP_BASE+i*4, 4);
        //decode the mapping
        //l:length bits, r:radio address bits, f:framework address bits -> llllllrr rrrrrrrr lllfffff ffffffff
        uint16_t mapping_radio_addr = (uint16_t)((eeprom_readings[0]&0b11)<<8) | (uint16_t)(eeprom_readings[1]);
        uint16_t mapping_length = (uint16_t)((eeprom_readings[0]&0b11111100)<<1) | (uint16_t)((eeprom_readings[2])>>5);
        //check if request address is in this mapping
        if((register_operation->address >= mapping_radio_addr) && (register_operation->address < mapping_radio_addr+mapping_length)) {
            uint16_t mapping_framework_addr = (uint16_t)((eeprom_readings[2]&0b11111)<<8) | (uint16_t)(eeprom_readings[3]);
            register_operation->address = mapping_framework_addr + (register_operation->address - mapping_radio_addr);
            return 1;
        }
    }
    //no mapping found
    return 0;
}

// ===== Radio Handler ===== //
//called when a request is received from the radio
void handler_radio(void) {
    //decode the radio packet
    reg_op register_operation;
    radio_decode(radio_buffer_rx, NRF905_PACKET_LENGTH, &register_operation);
    radio_flag_rx = 0;  //ready to read a new packet from the radio
    
    // === PIC register operation === //
    if(register_operation.address >= PIC_REG_BASE) {
        if(register_operation.type == REG_OP_READ_REQ) {
            pic_register_read(&register_operation);
            radio_send(&register_operation);
        }
        else if(register_operation.type == REG_OP_WRITE_REQ) {
            pic_register_write(&register_operation);
            radio_send(&register_operation);
        }
        else {
            error_state = ERROR_REG_PIC;
            return;
        }
        //a write operation just sends the value back as acknowledge (but it can be anything)
        //radio_send(&register_operation);
        return;
    }
    // == Indirect address register operation === //
    else {
        //save to later know what response to expect from the STM32
        reg_op_types radio_request_type = register_operation.type;

        //determines which operation should be done on the STM32
        if(!indirect_register_access(&register_operation)) {
            //abort if the radio address is not mapped to an STM32 address
            reg_map_status_read = MAP_ERROR_ACCESS;
            return;
        }
        //send the operation to the STM32
        framework_send(&register_operation);
        uart_flag_rx = 0;   //ready to receive UART packet
        //wait for the STM32 response
        uint16_t timeout = 0;
        while(!uart_flag_rx) {
            if(timeout++ > UART_TIMEOUT) {
                //error if no STM32 response
                error_state = ERROR_STM32_TIMEOUT;
                return;
            }
        }
        //verify the packet integrity
        if(!framework_verify(uart_buffer_rx, uart_buffer_rx_len)) {
            //error if received packet not valid
            error_state = ERROR_STM32_CHECKSUM;
            return;
        }
        //decode the framework response packet
        framework_decode(uart_buffer_rx, uart_buffer_rx_len, &register_operation);
        //send back a response to the radio
        if(radio_request_type == REG_OP_READ_REQ && register_operation.type == REG_OP_READ_RES) {
            radio_send(&register_operation);
        }
        else if(radio_request_type == REG_OP_WRITE_REQ && register_operation.type == REG_OP_WRITE_RES) {
            radio_send(&register_operation);
        }
        else {
            //STM32 response type not valid
            error_state = ERROR_STM32_RESPONSE;
            return;
        }
    }
}

void main(void) {
    // == Initialization == //
    adc_disable();
    spi_init();
    uart_init();
    led_init();
    
    //check that the NRF905 is connected correctly
    if(!nrf905_test()) {
        error_state = ERROR_SETUP;
    }
    
    //Reinitialize the EEPROM because it was reset by the PICKit4
    if(eeprom_read(EEPROM_ADDR_CHANNEL) == 0xFF && eeprom_read(EEPROM_ADDR_MAP_NB) == 0xFF) {
        eeprom_write(EEPROM_ADDR_CHANNEL, 81);  //default channel to 81
        eeprom_write(EEPROM_ADDR_MAP_NB, 0);    //reset the indirect address mappings
        nrf905_setup(81);
    }
    else {
        nrf905_setup(eeprom_read(EEPROM_ADDR_CHANNEL));
    }
    
    while(1) {
        //wait for a packet from the radio
        while(error_state==ERROR_OK) {
            while(!radio_flag_rx) {
                led_state(0);
            }
            led_state(1);
            handler_radio();
        }

        //show the error code 10 times
        for(uint8_t i=0;i<10;i++) {
            //blink as many times as the error code value
            for(uint8_t j=0;j<error_state;j++) {
                led_state(1);
                delay_us(200000);
                led_state(0);
                delay_us(200000);
            }
            //wait 1 seconds
            delay_us(1000000);
        }
        error_state = ERROR_OK;
        radio_flag_rx = 0;
        nrf905_setup(eeprom_read(EEPROM_ADDR_CHANNEL));
    }
    return;
}