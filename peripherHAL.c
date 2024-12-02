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
    PORTCbits.RC0 = state&1;
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
    SPBRG = 10;          //clock division to have a 57600 baudrate (actually 56818 but close enough, the STM32 head baudrate for the radio UART was set to 56800)
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
    PIE1bits.RCIE = 1;  //interrupt for RX
    
    INTCONbits.PEIE = 1;   //enable peripheral interruptions
    INTCONbits.GIE = 1;    //enable global interruptions
}

void uart_write(uint8_t data) {
    while(!TRMT);    //wait for transmit buffer to be empty
    TXREG = data;
}

void uart_write_buffer(uint8_t* data, uint8_t data_len) {
    for(uint8_t i=0;i<data_len;i++) {
        uart_write(data[i]);
    }
}

// ================== //
// ===== EEPROM ===== //
// ================== //
void eeprom_write(uint8_t address, uint8_t value) {
    while(EECON1bits.WR);    //wait for last write to finish if any
    EEADR = address;
    EEDATA = value;
    EECON1bits.EEPGD = 0;   //access data memory
    EECON1bits.WREN = 1;    //enable writting to the EEPROM
    INTCONbits.GIE = 0; //disable interrupts
    
    //sequence to start an eeprom write
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;   //start a write operation
    
    INTCONbits.GIE = 1;  //enable interrupts
    EECON1bits.WREN = 0; //disable writting to the EEPROM
}

uint8_t eeprom_read(uint8_t address) {
    EEADR = address;
    EECON1bits.EEPGD = 0;   //access data memory
    EECON1bits.RD = 1;  //start reading operation
    return EEDATA;
}

void eeprom_read_buffer(uint8_t* buffer, uint8_t address_base, uint8_t length) {
    for(uint8_t i=0;i<length;i++) {
        buffer[i] = eeprom_read(address_base+i);
    }
}

// ================ //
// ===== ADC ====== //
// ================ //

//all the PORT A pins are ADC inputs by default
//set all the ADC pins as Digital IO
void adc_disable(void) {
    ADCON1bits.PCFG0 = 0;
    ADCON1bits.PCFG1 = 1;
    ADCON1bits.PCFG2 = 1;
    ADCON1bits.PCFG3 = 0;
}