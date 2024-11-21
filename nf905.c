#include "nrf905.h"
#include "spi.h"
#include "utilities.h"

//Pins for the radio
//PWR_UP pin
#define TRIS_PWR_UP (TRISA4)
#define PIN_PWR_UP (RA4)
//TRX_CE pin
#define TRIS_TRX_CE (TRISC2)
#define PIN_TRX_CE (RC2)
//TX_EN pin
#define TRIS_TX_EN  (TRISC1)
#define PIN_TX_EN  (RC1)
//DR (TRX Data Ready) pin
#define TRIS_DR (TRISB0)
#define PIN_DR  (RB0)
//AM (Address Match)pin
#define TRIS_AM (TRISB1)
#define PIN_AM  (RB1)
//CD (Carrier Detect) pin
#define TRIS_CD (TRISB2)
#define PIN_CD  (RB2)

//defines for the NRF905 SPI operations
#define W_CONFIG        (0b00000000)
#define R_CONFIG        (0b00010000)
#define W_TX_PAYLOAD    (0b00100000)
#define R_TX_PAYLOAD    (0b00100001)
#define W_TX_ADDRESS    (0b00100010)
#define R_TX_ADDRESS    (0b00100011)
#define R_RX_PAYLOAD    0x24
#define CHANNEL_CONFIG  (0b10000000)

//dummy bye to transfer when doing an SPI read only
#define DUMMY_BYTE      (0x00)

//reads the 4 bytes of the default address to check that the nrf905 is connected, should be all be 0xE7
uint8_t nrf905_test(void) {
    uint8_t buffer[5];
    spi_cs(0);
    spi_xfer(R_CONFIG | 5);
    buffer[0] = spi_xfer(DUMMY_BYTE);
    buffer[1] = spi_xfer(DUMMY_BYTE);
    buffer[2] = spi_xfer(DUMMY_BYTE);
    buffer[3] = spi_xfer(DUMMY_BYTE);
    buffer[4] = spi_xfer(DUMMY_BYTE);
    spi_cs(1);
    if(buffer[0]==0xE7 && buffer[1]==0xE7 && buffer[2]==0xE7 && buffer[3]==0xE7) {
        return 1;
    }
    return 0;
}

void nrf905_setup(void) {
    //setup the pins
    TRIS_PWR_UP = 0;   //PWM_UP pin as output
    TRIS_TRX_CE = 0;   //TRX_CE pin as output
    TRIS_TX_EN = 0;    //TX_EN pin as output
    TRIS_DR = 1;       //DR pin as input
    TRIS_AM = 1;       //AM pin as input
    TRIS_CD = 1;       //CD pin as input
    nrf905_set_mode(NRF905_MODE_DOWN);  //Power down mode, to allow access to configuration registers
   
    //setup packet length
    spi_cs(0);
    spi_xfer(W_CONFIG | 3); //write the RX then TX length
    spi_xfer(NRF905_PACKET_LENGTH);
    spi_xfer(NRF905_PACKET_LENGTH);
    spi_cs(1);
    
    // setup config frequency (868/915MHz) and power (max)
    spi_cs(0);
    spi_xfer(W_CONFIG | 1);
    spi_xfer(0b00001110);
    spi_cs(1);
    
    //standby mode
    nrf905_set_mode(NRF905_MODE_STANDBY_RX);
    delay_us(3000);
}

//sets the power mode of the NRF905
void nrf905_set_mode(uint8_t mode) {
    PIN_PWR_UP = (mode>>2) & 1;
    PIN_TX_EN = (mode) & 1;
    PIN_TRX_CE = (mode>>1) & 1;
}

//send a packet, the length of payload should be NRF905_PACKET_LENGTH
void nrf905_send(uint8_t* payload) {
    nrf905_set_mode(NRF905_MODE_STANDBY_TX);
    spi_cs(0);
    spi_xfer(W_TX_PAYLOAD);
    for(uint8_t i=0;i<NRF905_PACKET_LENGTH;i++) {
        spi_xfer(payload[i]);
    }
    spi_cs(1);
    nrf905_set_mode(NRF905_MODE_TX);
    delay_us(10);
    nrf905_set_mode(NRF905_MODE_STANDBY_TX);
    while(PIN_DR == 0); //wait for packet to be sent
}

void nrf905_receive(uint8_t* payload) {
    nrf905_set_mode(NRF905_MODE_STANDBY_RX);
    spi_cs(0);
    spi_xfer(R_RX_PAYLOAD);
    for(uint8_t i=0;i<NRF905_PACKET_LENGTH;i++) {
        payload[i] = spi_xfer(DUMMY_BYTE);
    }
    spi_cs(1);
}