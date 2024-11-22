#include "peripherHAL.h"

// =============== //
// ===== SPI ===== //
// =============== //
void spi_init(void) {
    TRISCbits.TRISC3 = 0;   //CLK pin as output
    TRISCbits.TRISC5 = 0;   //MOSI pin as output
    
    TRISCbits.TRISC0 = 0;   //CS pin as output
    PORTCbits.RC0 = 1;      //CS pin high
    
    SSPSTATbits.SMP = 0;    //sample slave bit in the middle of master bit
    SSPSTATbits.CKE = 1;    //sample on clock rising edge
    
    SSPCONbits.SSPEN = 1;   //enable synchronous serial port
    SSPCONbits.CKP = 0;     //idle state for clock is low
    SSPCONbits.SSPM = 0b0000;   //SPI master mode, clock = Fosc/4
}

void spi_cs(uint8_t state) {
    PORTCbits.RC0 = state;
}

uint8_t spi_xfer(uint8_t data) {
    SSPBUF = data;
    while(!SSPSTATbits.BF); //wait for transfer to be done
    uint8_t output = SSPBUF;
    return output;
}

// ================ //
// ===== UART ===== //
// ================ //
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
    
    INTCONbits.PEIE = 1;   //enable peripheral interruptions
    INTCONbits.GIE = 1;    //enable global interruptions
}

void uart_write(uint8_t data) {
    while(!TRMT);    //wait for transmit buffer to be empty
    TXREG = data;
}

uint8_t uart_read(void) {
    while(RCIF == 0);   //wait for packet
    RCIF = 0;
    return RCREG;
}

// ================== //
// ===== EEPROM ===== //
// ================== //


// ================ //
// ===== ADC ====== //
// ================ //

//all the PORT A pins are ADC inputs by default
//set all the ADC pins as Digital IO
void adc_disable(void) {
    PCFG0 = 0;
    PCFG1 = 1;
    PCFG2 = 1;
    PCFG3 = 0;
}