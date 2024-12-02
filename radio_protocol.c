#include "radio_protocol.h"
#include "utilities.h"
#include "nrf905.h"

//convert a NRF905 raw payload to the reg_op (register operation) structure
void radio_decode(uint8_t* buffer, uint8_t size, reg_op* reg_op_buffer) {
    //retrieve the operation code and register address
    uint8_t op_code = buffer[0]>>2;
    reg_op_buffer->address = (uint16_t)((buffer[0] & 0b11)<<8);
    reg_op_buffer->address |= buffer[1];
    uint8_t buffer_shift = 2;   //index at which the register data starts
    
    //retrieve the register size from the OP code
    switch(op_code) {
        case 0x00:
        case 0x04:
            reg_op_buffer->size = 1;
            break;
        case 0x01:
        case 0x05:
            reg_op_buffer->size = 2;
            break;
        case 0x02:
        case 0x06:
            reg_op_buffer->size = 4;
            break;
        case 0x03:
        case 0x07:
            reg_op_buffer->size = buffer[2];
            buffer_shift = 3;   //the payload is 1 byte further because of the payload length also being transmitted
            break;
    }
    //retrieve the operation type
    if(op_code <= 0x03) {
        reg_op_buffer->type = REG_OP_READ_REQ;
    }
    else if(op_code <= 0x07) {
        reg_op_buffer->type = REG_OP_WRITE_REQ;
        //copy the register value
        for(uint8_t i=0;i<reg_op_buffer->size;i++) {
            reg_op_buffer->value[i] = buffer[buffer_shift + i];
        }
    }
}

//Only used to send responses to requests
//no protocol is involved for responses to radio requests
//just the raw register value is send back for a read request
void radio_send(reg_op* reg_op_buffer) {
    nrf905_send(reg_op_buffer->value, reg_op_buffer->size);
}