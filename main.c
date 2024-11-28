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

// === DEFINES === //
//version number stored in register 0x3E0
#define VERSION         0x49
#define PIC_REG_BASE    0x3E0

//radio channel
#define EEPROM_ADDR_CHANNEL     0x00
//number of mappings registered for indirect register access
#define EEPROM_ADDR_MAP_NB      0x01
//where the mappings storage starts (4 bytes per mapping)
#define EEPROM_ADDR_MAP_BASE    0x10

#define INDIRECT_MAPPINGS_MAX   50




// === Global Variables === //
// communication with the STM32 through UART
uint8_t uart_counter = 0;
uint8_t uart_buffer_rx[32];
uint8_t uart_buffer_rx_len = 7;
uint8_t uart_flag_rx = 0;

// communication with the client through the NRF905 (with SPI)
uint8_t radio_buffer_rx[32];
uint8_t radio_flag_rx = 0;

//indirect address control registers
uint8_t reg_indirect_status = 0;
uint16_t reg_indirect_addr_radio = 0;
uint16_t reg_indirect_addr_stm32 = 0;
uint16_t reg_indirect_len = 0;

//errors
#define ERROR_OK        0x00
#define ERROR_GENERIC   0x01
#define ERROR_SETUP     0x02
#define ERROR_REG_PIC   0x03
#define ERROR_REG_STM32 0x04
#define ERROR_REG_MAP   0x05

uint8_t error_state = ERROR_OK;

uint8_t debug[64] = {0xFF};
uint8_t debug_counter = 0;


// === Main ISR === //
void __interrupt() main_isr(void) {
    // === Interrupt from UART RX === //
    if(PIR1bits.RCIF) {
        uint8_t uart_value = RCREG;
        //state machine to decode the UART framework packet (find start bytes + payload length)
        if(uart_counter==0 && uart_value==0xC3 && uart_flag_rx==0) {
            uart_buffer_rx[uart_counter++] = uart_value;
        }
        else if(uart_counter>=1) {
            if(uart_counter==1 && uart_value!=0x3C) {
                uart_counter = 0;
                return;
            }
            else if(uart_counter==3) {
                uart_buffer_rx_len = uart_value + 6;   //index at which the packet will stop: Start bytes(2) + Source (1) + Length(1) + Checksum(1) + End(1)
            }
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
void pic_register_read(reg_op* register_operation) {
    switch(register_operation->address) {
        case 0x3E0: //992
            register_operation->value[0] = VERSION;
            register_operation->size = 1;
            return;
        case 0x3E1: //993
            register_operation->value[0] = eeprom_read(EEPROM_ADDR_CHANNEL);
            register_operation->size = 1;
            return;
        case 0x3E2: //994
            register_operation->value[0] = eeprom_read(EEPROM_ADDR_MAP_NB);
            register_operation->size = 1;
            return;
        case 0x3E3: //995
            register_operation->value[0] = reg_indirect_addr_radio&0xFF;
            register_operation->value[1] = reg_indirect_addr_radio>>8;
            register_operation->size = 2;
            return;
        case 0x3E4: //996
            register_operation->value[0] = reg_indirect_addr_stm32&0xFF;
            register_operation->value[1] = reg_indirect_addr_stm32>>8;
            register_operation->size = 2;
            return;
        case 0x3E5: //997
            register_operation->value[0] = reg_indirect_len&0xFF;
            register_operation->value[1] = reg_indirect_len>>8;
            register_operation->size = 2;
            return;
    }
}

void pic_register_write(reg_op* register_operation) {
    switch(register_operation->address) {
        case 0x3E0: //992
            //send carrier
            if(register_operation->value[0] == 0xAA) {}
            //0x10 - 0x13 set radio power
            else if(register_operation->value[0]) {}
            return;
            
        case 0x3E1: //993
            eeprom_write(EEPROM_ADDR_CHANNEL, register_operation->value[0]);
            nrf905_set_channel(register_operation->value[0]);
            return;
            
        case 0x3E2: //994
            //create a new mapping
            //l: length bits, r:radio bits, f:framework bits -> llllllrr rrrrrrrr lllfffff ffffffff
            if(register_operation->value[0] == 0xAA && reg_indirect_len > 0) {
                uint8_t mappings_count = eeprom_read(EEPROM_ADDR_MAP_NB);
                if(mappings_count >= INDIRECT_MAPPINGS_MAX) {
                    error_state = ERROR_REG_MAP;
                    return;
                }
                eeprom_write(EEPROM_ADDR_MAP_BASE + mappings_count*4, ((reg_indirect_len>>1)&0b11111100) | ((reg_indirect_addr_radio>>8)&0b00000011));
                eeprom_write(EEPROM_ADDR_MAP_BASE + mappings_count*4 + 1, reg_indirect_addr_radio&0xFF);
                eeprom_write(EEPROM_ADDR_MAP_BASE + mappings_count*4 + 2, ((reg_indirect_len<<5)&0b11100000) | ((reg_indirect_addr_stm32>>8)&0b00011111));
                eeprom_write(EEPROM_ADDR_MAP_BASE + mappings_count*4 + 3, reg_indirect_addr_stm32&0xFF);
                eeprom_write(EEPROM_ADDR_MAP_NB, ++mappings_count);
                reg_indirect_len = 0;
                return;
            }
            else if(register_operation->value[0] == 0xAC && reg_indirect_status == 0xAB) {
                eeprom_write(EEPROM_ADDR_MAP_NB, 0);
            }
            reg_indirect_status = register_operation->value[0];
            return;
            
        case 0x3E3: //995
            reg_indirect_addr_radio = ((register_operation->value[1]&0b00000011)<<8) | register_operation->value[0];
            return;
            
        case 0x3E4: //996
            reg_indirect_addr_stm32 = ((register_operation->value[1]&0b000111111)<<8) | register_operation->value[0];
            return;
            
        case 0x3E5: //997
            reg_indirect_len = ((register_operation->value[1]&0b00000001)<<8) | register_operation->value[0];
            return;
    }
}


// ===== Indirect registers ===== //
void indirect_register_access(reg_op* register_operation) {
    uint8_t mappings_nb = eeprom_read(EEPROM_ADDR_MAP_NB);
    uint8_t eeprom_readings[4];
    for(uint8_t i=0;i<mappings_nb;i++) {
        //read a mapping from the EEPROM
        eeprom_read_buffer(eeprom_readings, EEPROM_ADDR_MAP_BASE+i*4, 4);
        //decode the mapping
        //l: length bits, r:radio bits, f:framework bits -> llllllrr rrrrrrrr lllfffff ffffffff
        uint16_t mapping_radio_addr = (uint16_t)((eeprom_readings[0]&0b11)<<8) | (uint16_t)(eeprom_readings[1]);
        uint16_t mapping_length = (uint16_t)((eeprom_readings[0]&0b11111100)<<1) | (uint16_t)((eeprom_readings[2])>>5);
        //check if request address is in this mapping
        if((register_operation->address >= mapping_radio_addr) && (register_operation->address < mapping_radio_addr+mapping_length)) {
            uint16_t mapping_framework_addr = (uint16_t)((eeprom_readings[2]&0b11111)<<8) | (uint16_t)(eeprom_readings[3]);
            register_operation->address = mapping_framework_addr + (register_operation->address - mapping_radio_addr);
            return;
        }
    }
    //no mapping found
    //error_state = ERROR_REG_MAP;
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
        }
        else if(register_operation.type == REG_OP_WRITE_REQ) {
            pic_register_write(&register_operation);
        }
        else {
            error_state = ERROR_REG_PIC;
            return;
        }
        //a write operation just sends the value back as acknowledge (but it can be anything)
        radio_send(&register_operation);
        return;
    }
    // == Indirect address register operation === //
    else {
        //save to later know what response to expect from the STM32
        reg_op_types radio_request_type = register_operation.type;
        if(register_operation.address < 63) {
            debug[register_operation.address] = 0xAA;
        }
        else {
            debug[63] = 0xBB;
        }
        //determines which operation should be done on the STM32
        indirect_register_access(&register_operation);
        if(error_state != ERROR_OK) {
            return;
        }
        //send the operation to the STM32
        framework_send(&register_operation);
        uart_flag_rx = 0;
        //wait for the STM32 response
        while(!uart_flag_rx);
        //verify the packet integrity
        if(!framework_verify(uart_buffer_rx, uart_buffer_rx_len)) {
            error_state = ERROR_REG_STM32;
            return;
        }
        //decode the framework response packet
        framework_decode(uart_buffer_rx, uart_buffer_rx_len, &register_operation);
        uart_flag_rx = 0;   //reset the UART flag
        //send back a response to the radio and check the response packet
        if(radio_request_type == REG_OP_READ_REQ && register_operation.type == REG_OP_READ_RES) {
            radio_send(&register_operation);
        }
        else if(radio_request_type == REG_OP_WRITE_REQ && register_operation.type == REG_OP_WRITE_RES) {
            radio_send(&register_operation);
        }
        //STM32 response not valid
        else {
            //error_state = ERROR_REG_STM32;
        }
    }
}

void main(void) {
    // == Initialization == //
    adc_disable();
    spi_init();
    uart_init();
    led_init();
    
    //Reinitialize the EEPROM because it was reset by the PICKit4
    if(eeprom_read(EEPROM_ADDR_CHANNEL) == 0xFF && eeprom_read(EEPROM_ADDR_MAP_NB) == 0xFF) {
        eeprom_write(EEPROM_ADDR_CHANNEL, 81);
        eeprom_write(EEPROM_ADDR_MAP_NB, 0);
    }
    
    //check that the NRF905 is connected correctly
    if(!nrf905_test()) {
        error_state = ERROR_SETUP;
    }
    nrf905_setup(81);
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
    }
    return;
}