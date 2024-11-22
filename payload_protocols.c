#include "payload_protocols.h"

#define PAYLOAD_TYPE_WREQ    0b01000000
#define PAYLOAD_TYPE_RREQ    0b01100000
#define PAYLOAD_TYPE_WRES    0b10000000
#define PAYLOAD_TYPE_RRES    0b10100000
#define PAYLOAD_TYPE_ERR     0b11000000
#define PAYLOAD_TYPE_PUB     0b00000000

uint8_t payload_checksum(uint8_t* payload, uint8_t len) {
    uint8_t checksum = 0;
    for(uint8_t i=0;i<len-2;i++) {
        checksum += payload[i];
    }
    checksum = (~checksum)+1;   //two's complement
    return checksum;
}

uint8_t payload_verify(uint8_t* payload, uint8_t len) {
    if(payload[len-1] == 0xFF) {
        if(payload[len-2] == payload_checksum(payload, len)) {
            return 1;
        }
    }
    return 0;
}

//add the start bytes, source address, payload length, checksum and stop byte
//Must be called after the register data have been put into the payload
void payload_build_frame(uint8_t* payload, uint8_t len) {
    //start bytes
    payload[0] = 0xC3;
    payload[1] = 0x3C;
    //source address
    payload[2] = PIC_ADDRESS;
    //payload length
    payload[3] = len - 6;
    //checksum
    payload[len-2] = payload_checksum(payload, len);
    //stop byte
    payload[len-1] = 0xFF;
}

void payload_build_wreq(uint8_t* payload, uint8_t* len, uint16_t register_address, uint8_t* register_value, uint8_t register_size) {
    len* = 8 + register_size;
    payload[4] = PAYLOAD_TYPE_WREQ | ((register_address>>8)&0b11111);
    payload[5] = (register_address&0xFF);
    for(uint8_t i=0;i<register_size;i++) {
        payload[6+i] = register_value[0];
    }
}