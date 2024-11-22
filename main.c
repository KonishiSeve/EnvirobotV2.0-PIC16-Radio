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
#include "payload_protocols.h"


uint8_t counter = 0;
uint8_t uart_packet[32];
uint8_t uart_packet_len = 7;
uint8_t uart_flag_rx = 0;

//main ISR, reading the interrupt flags and calling the correct subroutine
void __interrupt() main_isr(void) {
    //if a UART byte was received
    if(PIR1bits.RCIF) {
        uint8_t uart_value = RCREG;
        if(counter==0 && uart_value==0xC3 && uart_flag_rx==0) {
            uart_packet[counter++] = uart_value;
        }
        else if(counter>=1) {
            if(counter==1 && uart_value!=0x3C) {
                counter = 0;
                return;
            }
            else if(counter==3) {
                uart_packet_len = uart_value + 6;   //index at the packet will stop (and the byte be 0xFF), Start bytes(2) + Source (1) + Length(1) + Checksum(1) + End(1)
            }
            else if(counter >= uart_packet_len-1) {
                uart_flag_rx = 1;
                uart_packet[counter] = uart_value;
                counter = 0;
                return;
            }
            uart_packet[counter++] = uart_value;
        }
    }
}

void main(void) {
    // == Initialization == //
    adc_disable();
    spi_init();
    uart_init();
    led_init();
    
    while(1) {
        led_state(0);
        uart_write(0xC3);
        uart_write(0x3C);
        uart_write(0xFE);
        uart_write(0x02);
        uart_write(0x60);
        uart_write(0x00);
        uart_write(0XA1);
        uart_write(0xFF);
        while(!uart_flag_rx);
        if(payload_verify(uart_packet, uart_packet_len)) {
            led_state(1);
        }
        uart_flag_rx = 0;
        delay_us(500000);
    }
    
    //wait for the nrf905 to be detected
    uint8_t loop = 1;
    while(loop) {
        if(!nrf905_test()) {
            led_state(0);
            loop = 0;
            break;
        }
        //led_state(1);
        delay_us(500000);
        //led_state(0);
        delay_us(500000);
    }
    
    // Radio setup
    nrf905_setup();
    delay_us(1000);
    uint8_t buffer[NRF905_PACKET_LENGTH];
    
    // == Main Loop == //
    while(1) {
        
        /*
        nrf905_set_mode(NRF905_MODE_RX);
        while(!RB0) {
            led_state(0);
        }
        nrf905_receive(buffer);
        if(buffer[0] == 0x03 && buffer[1] == 0xe0) {
            led_state(1);
        }
        buffer[0] = 0x50;
        nrf905_send(buffer);
        delay_us(500000);
         */
    }
    return;
}
