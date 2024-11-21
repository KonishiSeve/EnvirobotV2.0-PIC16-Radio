#include "spi.h"

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