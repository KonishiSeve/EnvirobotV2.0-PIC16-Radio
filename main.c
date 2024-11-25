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
uint8_t radio_buffer_tx[32];
uint8_t radio_flag_rx = 0;

// for the global state machine
reg_op register_operation_rx;
reg_op register_operation_tx;
uint8_t error_state = 0;

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
        if(!radio_flag_rx) {
            nrf905_receive(radio_buffer_rx);
            radio_flag_rx = 1;
        }
    }
}

// ===== Radio Handler ===== //
//called when a request is received from the radio
void handler_radio(void) {
    //decode the radio packet
    radio_decode(radio_buffer_rx, NRF905_PACKET_LENGTH, &register_operation_rx);
    radio_flag_rx = 0;
    
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

    //send back a response to the radio
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

void main(void) {
    // == Initialization == //
    adc_disable();
    spi_init();
    uart_init();
    led_init();
    
    //check that the NRF905 is connected correctly
    if(!nrf905_test()) {
        error_state = 1;
    }
    nrf905_setup();
    
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