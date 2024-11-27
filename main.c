/*
 * File:   main.c
 * Author: skonishi
 *
 * Created on 15. novembre 2024, 16:23
 */
// CONFIG
#pragma config FOSC = HS        // Oscillator Selection bits (HS oscillator)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = OFF       // Brown-out Reset Enable bit (BOR enabled)
#pragma config LVP = OFF         // Low-Voltage (Single-Supply) In-Circuit Serial Programming Enable bit (RB3/PGM pin has PGM function; low-voltage programming enabled)
#pragma config CPD = OFF        // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
#pragma config WRT = OFF        // Flash Program Memory Write Enable bits (Write protection off; all program memory may be written to by EECON control)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define VERSION 0x49

#include <xc.h>
#include "nrf905.h"
#include "utilities.h"
#include "peripherHAL.h"
#include "framework_protocol.h"
#include "radio_protocol.h"


// === Global Variables === //
// communication with the STM32 through UART
uint8_t uart_counter = 0;
uint8_t uart_buffer_rx[32];
uint8_t uart_buffer_rx_len = 7;
uint8_t uart_buffer_tx[32];
uint8_t uart_buffer_tx_len = 0;
uint8_t uart_flag_rx = 0;

// communication with the client through the NRF905 (with SPI)
uint8_t radio_buffer_rx[32];
uint8_t radio_flag_rx = 0;

// for the global state machine
reg_op register_operation_rx;
reg_op register_operation_tx;
uint8_t error_state = 0;

//RADIO REMOTE DEBUG
uint32_t history[20];
uint8_t history_counter = 0;
uint8_t debug_counter = 0;

// === Main ISR === //
void __interrupt() main_isr(void) {
    // === Interrupt from UART RX === //
    if(PIR1bits.RCIF) {
        uint8_t uart_value = RCREG;
        if(uart_counter==0 && uart_value==0xC3 && uart_flag_rx==0) {
            uart_buffer_rx[uart_counter++] = uart_value;
        }
        else if(uart_counter>=1) {
            if(uart_counter==1 && uart_value!=0x3C) {
                uart_counter = 0;
                return;
            }
            else if(uart_counter==3) {
                uart_buffer_rx_len = uart_value + 6;   //index at the packet will stop (and the byte be 0xFF), Start bytes(2) + Source (1) + Length(1) + Checksum(1) + End(1)
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
        INTCONbits.INTF = 0;    //clear the flag
        debug_counter++;
        if(!radio_flag_rx) {
            nrf905_receive(radio_buffer_rx);
            radio_flag_rx = 1;
        }
    }
}
// ===== PIC registers ===== //
void radio_register_read_byte(uint16_t address) {
    register_operation_tx.size = 1;
    switch(address) {
        case 0x3E0:
            register_operation_tx.value[0] = VERSION;
            return;
        case 0x3E1:
            register_operation_tx.value[0] = eeprom_read(0x00);
            return;
    }
}


// ===== Radio Handler ===== //
//called when a request is received from the radio
void handler_radio(void) {
    //Radio logging
    uint32_t temp = radio_buffer_rx[0];
    uint32_t temp2 = radio_buffer_rx[1];
    uint32_t temp3 = radio_buffer_rx[2];
    history[history_counter++] = temp<<16 | temp2<<8 | temp3;
    
    //decode the radio packet
    radio_decode(radio_buffer_rx, NRF905_PACKET_LENGTH, &register_operation_rx);
    radio_flag_rx = 0;
    
    //local PIC register
    if(register_operation_rx.address >= 0x3E0) {
        if(register_operation_rx.type == REG_OP_READ_REQ) {
            if(register_operation_rx.size == 1) {
                radio_register_read_byte(register_operation_rx.address);
            }
        }
        nrf905_send(register_operation_tx.value, register_operation_tx.size);
    }
    
    //Radio Remote Hacking
    else if (register_operation_rx.address <= 0xFF) {
        static radio_hack_0 = 7;
        if(register_operation_rx.address == 0x00) { //mode
            if(register_operation_rx.type==REG_OP_READ_REQ) {
                register_operation_tx.size = 1;
                register_operation_tx.value[0] = radio_hack_0;
            }
            else {
                if(radio_hack_0 == 1) {
                    radio_hack_0 = 0;
                }
                else if(radio_hack_0 == 0) {
                    radio_hack_0 = 3;
                }
            }
        }
        else if(register_operation_rx.address == 0x01 && register_operation_rx.type==REG_OP_READ_REQ) {    //number of elements
            register_operation_tx.size = 1;
            register_operation_tx.value[0] = 4;
        }
        else {
            register_operation_tx.size = 4;
        }
        
        nrf905_send(register_operation_tx.value, register_operation_tx.size);
    }
    else {
        //encode the register operation to a UART packet for the framework
        framework_encode(uart_buffer_tx, &uart_buffer_tx_len, &register_operation_rx);
        //send the packet to the STM32
        uart_write_buffer(uart_buffer_tx, uart_buffer_tx_len);
        //wait for the STM32 response
        while(!uart_flag_rx);
        //verify the packet integrity
        if(!framework_verify(uart_buffer_rx, uart_buffer_rx_len)) {
            error_state = 1;
            return;
        }
        //decode the framework response packet
        framework_decode(uart_buffer_rx, uart_buffer_rx_len, &register_operation_tx);
        uart_flag_rx = 0;   //reset the UART flag
        //send back a response to the radio and check the response packet
        if(register_operation_rx.type == REG_OP_READ_REQ && register_operation_tx.type == REG_OP_READ_RES) {
            nrf905_send(register_operation_tx.value, register_operation_tx.size);
        }
        else if(register_operation_rx.type == REG_OP_WRITE_REQ && register_operation_tx.type == REG_OP_WRITE_RES) {
            nrf905_send(register_operation_tx.value, register_operation_tx.size);
        }
        //STM32 response not valid
        else {
            error_state = 1;
        }
    }
}

void main(void) {
    // == Initialization == //
    eeprom_write(0x00, 81);
    adc_disable();
    spi_init();
    uart_init();
    led_init();
    
    if(eeprom_read(0x00) != 81) {
        error_state = 1;
    }
    
    //check that the NRF905 is connected correctly
    if(!nrf905_test()) {
        error_state = 1;
    }
    nrf905_setup();
    nrf905_set_channel(eeprom_read(0x00));
    while(1) {
        //wait for a packet from the radio
        while(!error_state) {
            while(!radio_flag_rx) {
                led_state(0);
            }
            led_state(1);
            handler_radio();
        }

        //blink the LED for 10 seconds when an error occurred
        for(uint8_t i=0;i<20;i++) {
            led_state(1);
            delay_us(250000);
            led_state(0);
            delay_us(250000);
        }
        error_state = 0;
    }
    return;
}