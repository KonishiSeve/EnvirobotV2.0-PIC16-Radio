#include "nrf905.h"
#include "peripherHAL.h"
#include "utilities.h"

// == Pins for the radio == //
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

// == defines for the NRF905 SPI operations == //
#define W_CONFIG        (0b00000000)
#define R_CONFIG        (0b00010000)
#define W_TX_PAYLOAD    (0b00100000)
#define R_TX_PAYLOAD    (0b00100001)
#define W_TX_ADDRESS    (0b00100010)
#define R_TX_ADDRESS    (0b00100011)
#define R_RX_PAYLOAD    (0b00100100)
#define CHANNEL_CONFIG  (0b10000000)

//dummy bye to transfer when doing an SPI read only
#define DUMMY_BYTE      (0x00)

//reads the 4 bytes of the default address to check that the nrf905 is connected, should all be 0xE7
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

//configures the nrf905
void nrf905_setup(uint8_t channel) {
    //setup the pins
    TRIS_PWR_UP = 0;   //PWM_UP pin as output
    TRIS_TRX_CE = 0;   //TRX_CE pin as output
    TRIS_TX_EN = 0;    //TX_EN pin as output
    TRIS_DR = 1;       //DR pin as input
    TRIS_AM = 1;       //AM pin as input
    TRIS_CD = 1;       //CD pin as input
    
    //standby
    PIN_PWR_UP = 1;
    PIN_TRX_CE = 0;
    PIN_TX_EN = 0;
    delay_us(3000); //wait for the nrf905 to switch to standby mode
   
    //setup packet length
    spi_cs(0);
    spi_xfer(W_CONFIG | 3); //write the RX then TX length
    spi_xfer(NRF905_PACKET_LENGTH);
    spi_xfer(NRF905_PACKET_LENGTH);
    spi_cs(1);
    
    // setup channel, frequency (868/915MHz) and power (max)
    spi_cs(0);
    spi_xfer(W_CONFIG | 0);
    spi_xfer(channel);
    spi_xfer(0b00001110);
    spi_cs(1);
    
    //RX mode
    PIN_PWR_UP = 1;
    PIN_TRX_CE = 1;
    PIN_TX_EN = 0;
    delay_us(650);  //wait for the nrf905 to switch to RX mode
    
    //enable interrupts on DR pin
    INTCONbits.INTE = 1;
}

//change the NRF905 channel
void nrf905_set_channel(uint8_t channel) {
    //standby mode
    PIN_PWR_UP = 1;
    PIN_TRX_CE = 0;
    PIN_TX_EN = 0;
    
    spi_cs(0);
    spi_xfer(W_CONFIG | 0);
    spi_xfer(channel);
    spi_cs(1);
    
    //RX mode
    PIN_PWR_UP = 1;
    PIN_TRX_CE = 1;
    PIN_TX_EN = 0;
    delay_us(650);
}

//send a packet with the NRF905, the payload is 0 padded if payload_len is < NRF905_PACKET_LENGTH
void nrf905_send(uint8_t* payload, uint8_t payload_len) {
    //disable interrupts on DR pin
    INTCONbits.INTE = 0;
    
    //standby mode
    PIN_PWR_UP = 1;
    PIN_TRX_CE = 0;
    PIN_TX_EN = 1;
    
    //writing the payload to the radio
    spi_cs(0);
    spi_xfer(W_TX_PAYLOAD);
    for(uint8_t i=0;i<NRF905_PACKET_LENGTH;i++) {
        if(i >= payload_len) {
            //fill with DUMMY bytes if less than NRF905_PACKET_LENGTH bytes in the payload
            spi_xfer(DUMMY_BYTE);
        }
        else {
            spi_xfer(payload[i]);
        }
    }
    spi_cs(1);
    //Trigger the radio transmitter
    PIN_TRX_CE = 1;
    delay_us(20);
    PIN_TRX_CE = 0;
    //wait for packet to be sent
    while(PIN_DR == 0);
    //switch back to RX mode (and clear the DR pin)
    PIN_TX_EN = 0;
    PIN_TRX_CE = 1;
    
    INTCONbits.INTF = 0;    //clear the flag
    INTCONbits.INTE = 1;    //enable interrupt on DR pin
}

//Read the RX payload of the NRF905 radio
void nrf905_receive(uint8_t* payload) {
    //standby mode
    PIN_PWR_UP = 1;
    PIN_TRX_CE = 0;
    PIN_TX_EN = 0;
    
    spi_cs(0);
    spi_xfer(R_RX_PAYLOAD);
    for(uint8_t i=0;i<NRF905_PACKET_LENGTH;i++) {
        payload[i] = spi_xfer(DUMMY_BYTE);
    }
    spi_cs(1);
    //RX mode
    PIN_TRX_CE = 1;
}