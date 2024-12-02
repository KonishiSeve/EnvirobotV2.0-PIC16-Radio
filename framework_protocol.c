#include "framework_protocol.h"
#include "peripherHAL.h"

//operation bits
#define PAYLOAD_TYPE_WREQ    0b01000000
#define PAYLOAD_TYPE_RREQ    0b01100000
#define PAYLOAD_TYPE_WRES    0b10000000
#define PAYLOAD_TYPE_RRES    0b10100000
#define PAYLOAD_TYPE_ERR     0b11000000
#define PAYLOAD_TYPE_PUB     0b00000000

//compute the checksum byte for a payload
uint8_t framework_checksum(uint8_t* buffer, uint8_t buffer_len) {
    uint8_t checksum = 0;
    //take all bytes into account except checksum and stop byte
    for(uint8_t i=0;i<buffer_len-2;i++) {
        checksum += buffer[i];
    }
    checksum = (~checksum)+1;   //two's complement
    return checksum;
}

//verify the integrity of a payload, with end byte and checksum
//the start bytes are already used/checked for synchronization at reception
uint8_t framework_verify(uint8_t* buffer, uint8_t buffer_len) {
    if(buffer[buffer_len-1] == 0xFF) {  //check that last byte is a stop byte
        if(buffer[buffer_len-2] == framework_checksum(buffer, buffer_len)) {
            return 1;
        }
    }
    return 0;
}

//send a read or write request framework packet to the STM32
//no buffer is used to save RAM
void framework_send(reg_op* reg_op_buffer) {
    //start bytes
    uart_write(0xC3);
    uart_write(0x3C);
    
    //source address
    uart_write(PIC_ADDRESS);
    uint8_t checksum = (uint8_t)(0xC3 + 0x3C + PIC_ADDRESS);
    
    //read request
    if(reg_op_buffer->type == REG_OP_READ_REQ) {
        //payload length
        uart_write(2); //2 bytes for register address and operation bits
        checksum += 2;
        //operation bits and address upper bits
        uart_write(PAYLOAD_TYPE_RREQ | ((reg_op_buffer->address>>8)&0b11111));
        checksum += PAYLOAD_TYPE_RREQ | ((reg_op_buffer->address>>8)&0b11111);
        //address LSB
        uart_write(reg_op_buffer->address&0xFF);
        checksum += reg_op_buffer->address&0xFF;
    }
    //write request
    else if(reg_op_buffer->type == REG_OP_WRITE_REQ) {
        uart_write(reg_op_buffer->size + 2); //2 bytes for register address and operation bits
        checksum += reg_op_buffer->size + 2;
        //operation bits and address upper bits
        uart_write(PAYLOAD_TYPE_WREQ | ((reg_op_buffer->address>>8)&0b11111));
        checksum += PAYLOAD_TYPE_WREQ | ((reg_op_buffer->address>>8)&0b11111);
        //address LSB
        uart_write(reg_op_buffer->address&0xFF);
        checksum += reg_op_buffer->address&0xFF;
        //payload body
        for(uint8_t i=0;i<reg_op_buffer->size;i++) {
            uart_write(reg_op_buffer->value[reg_op_buffer->size - i - 1]);
            checksum += reg_op_buffer->value[reg_op_buffer->size - i - 1];
        }
    }
    //checksum byte
    checksum = (~checksum)+1;   //two's complement
    uart_write(checksum);
    //stop byte
    uart_write(0xFF);
}

//convert a raw UART payload to the reg_op (register operation) structure
void framework_decode(uint8_t* buffer, uint8_t buffer_len, reg_op* reg_op_buffer) {
    //retrieve the operation type
    switch(buffer[4] & 0b11100000) {
        case PAYLOAD_TYPE_WREQ:
            reg_op_buffer->type = REG_OP_WRITE_REQ;
            break;
        case PAYLOAD_TYPE_RREQ:
            reg_op_buffer->type = REG_OP_READ_REQ;
            break;
        case PAYLOAD_TYPE_RRES:
            reg_op_buffer->type = REG_OP_READ_RES;
            break;
        case PAYLOAD_TYPE_WRES:
            reg_op_buffer->type = REG_OP_WRITE_RES;
            break;
    }
    //retrieve register size
    reg_op_buffer->size = buffer_len - 8;
    //retrieve the register address
    reg_op_buffer->address = (uint16_t)(buffer[4]&0b11111)<<8;
    reg_op_buffer->address |= buffer[5];
    //retrieve the register value
    for(uint8_t i=0;i<reg_op_buffer->size;i++) {
        reg_op_buffer->value[reg_op_buffer->size - i - 1] = buffer[6+i];
    }
}