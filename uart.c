#include "uart.h"

void uart_init(void) {
    TRISCbits.TRISC6 = 1;   //set RC6 to allow the UART peripheral to use it
    TRISCbits.TRISC7 = 1;   //set RC7 to allow the UART peripheral to use it
    
    //baudrate configuration
    SPBRG = 10;          //clock division to have a 57600 baudrate (actually 56818 but close enough)
    TXSTAbits.BRGH = 1; //select baud rate as FOSC/(16(SPBRG + 1)) with FOSC = 10MHz
    
    //TX configuration
    TXSTAbits.TX9 = 0;  //8bits mode
    TXSTAbits.TXEN = 1; //enable TX
    TXSTAbits.SYNC = 0; //asynchronous mode
    
    //RX configuration
    RCSTAbits.SPEN = 1; //enable serial port
    RCSTAbits.RX9 = 0;  //8bits mode
    RCSTAbits.CREN = 1; //continuous receive
    RCSTAbits.ADDEN = 0;//disable address detection
    
    //enable UART interrupts
    //PIE1bits.TXIE = 1;  //interrupt for TX
    PIE1bits.RCIE = 1;  //interrupt for RX
}

void uart_write(uint8_t data) {
    TXREG = data;
}

uint8_t uart_read(void) {
    return RCREG;
}