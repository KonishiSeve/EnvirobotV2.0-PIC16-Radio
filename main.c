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
#include "spi.h"
#include "uart.h"
#include "nrf905.h"
#include "utilities.h"

/*
void uart_isr(void) {
}

void nrf905_isr(void) {
    
}

//main ISR, reading the interrupt flags and calling the correct subroutine
void __interrupt() main_isr(void) {
}
*/

void main(void) {
    // == Setup == //
    spi_init();
    
    TRISAbits.TRISA5 = 0;   //set LED pin as output
    PORTAbits.RA5 = 1;      //turn LED on
    
    //Set all A PORT pins to Digital IO (and not the default Analog Input)
    PCFG0 = 0;
    PCFG1 = 1;
    PCFG2 = 1;
    PCFG3 = 0;
    
    //for(volatile uint32_t i=0;i<10000;i++);
    uint8_t loop = 1;
    
    //wait for the sensor to be detected
    while(loop) {
        if(nrf905_test()) {
            PORTAbits.RA5 = 0;      //turn LED on
            loop = 0;
            break;
        }
        PORTAbits.RA5 = 1;      //turn LED on
        delay_us(500000);
        PORTAbits.RA5 = 0;      //turn LED off
        delay_us(500000);
    }
    
    nrf905_setup();
    delay_us(1000);
    uint8_t buffer[NRF905_PACKET_LENGTH];
    while(1) {
        nrf905_set_mode(NRF905_MODE_RX);
        while(!RB0) {
            PORTAbits.RA5 = 0;      //turn LED on
        }
        nrf905_receive(buffer);
        if(buffer[0] == 0x03 && buffer[1] == 0xe0) {
            PORTAbits.RA5 = 1;      //turn LED on
        }
        buffer[0] = 0x50;
        nrf905_send(buffer);
        delay_us(500000);
    }
    return;
}
