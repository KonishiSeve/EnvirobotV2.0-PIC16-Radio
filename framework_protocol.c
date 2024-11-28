#include "framework_protocol.h"
#include "peripherHAL.h"

#define PAYLOAD_TYPE_WREQ    0b01000000
#define PAYLOAD_TYPE_RREQ    0b01100000
#define PAYLOAD_TYPE_WRES    0b10000000
#define PAYLOAD_TYPE_RRES    0b10100000
#define PAYLOAD_TYPE_ERR     0b11000000
#define PAYLOAD_TYPE_PUB     0b00000000

#define PAYLOAD_FLAG_OK      0x00

//compute the checksum byte for a payload
uint8_t framework_checksum(uint8_t* buffer, uint8_t buffer_len) {
    uint8_t checksum = 0;
    for(uint8_t i=0;i<buffer_len-2;i++) {
        checksum += buffer[i];
    }
    checksum = (~checksum)+1;   //two's complement
    return checksum;
}

//verify the integrity of a payload, with end byte and checksum
uint8_t framework_verify(uint8_t* buffer, uint8_t buffer_len) {
    if(buffer[buffer_len-1] == 0xFF) {
        if(buffer[buffer_len-2] == framework_checksum(buffer, buffer_len)) {
            return 1;
        }
    }
    return 0;
}

void framework_send(reg_op* reg_op_buffer) {
    //start bytes
    uart_write(0xC3);
    uart_write(0x3C);
    //source address
    uart_write(PIC_ADDRESS);
    uint8_t checksum = (uint8_t)(0xC3 + 0x3C + PIC_ADDRESS);
    if(reg_op_buffer->type == REG_OP_READ_REQ) {
        //payload length
        uart_write(2); //2 bytes for register address and operation bits
        checksum += 2;
        volatile uint16_t value = (reg_op_buffer->address>>8);
        volatile uint16_t value2 = ((reg_op_buffer->address)>>8);
        uart_write(PAYLOAD_TYPE_RREQ | ((reg_op_buffer->address>>8)&0b11111));
        checksum += PAYLOAD_TYPE_RREQ | ((reg_op_buffer->address>>8)&0b11111);
        uart_write(reg_op_buffer->address&0xFF);
        checksum += reg_op_buffer->address&0xFF;
    }
    else if(reg_op_buffer->type == REG_OP_WRITE_REQ) {
        uart_write(reg_op_buffer->size + 2); //2 bytes for register address and operation bits
        checksum += reg_op_buffer->size + 2;
        uart_write(PAYLOAD_TYPE_WREQ | ((reg_op_buffer->address>>8)&0b11111));
        checksum += PAYLOAD_TYPE_WREQ | ((reg_op_buffer->address>>8)&0b11111);
        uart_write(reg_op_buffer->address&0xFF);
        checksum += reg_op_buffer->address&0xFF;
        //payload body
        for(uint8_t i=0;i<reg_op_buffer->size;i++) {
            uart_write(reg_op_buffer->value[reg_op_buffer->size - i - 1]);
            checksum += reg_op_buffer->value[reg_op_buffer->size - i - 1];
        }
    }
    checksum = (~checksum)+1;   //two's complement
    uart_write(checksum);
    //stop byte
    uart_write(0xFF);
}

/*
//register operation --> UART buffer
void framework_encode(uint8_t* buffer, uint8_t* buffer_len, reg_op* reg_op_buffer) {
    //start bytes
    buffer[0] = 0xC3;
    buffer[1] = 0x3C;
    //source address
    buffer[2] = PIC_ADDRESS;

    //payload length and header
    if(reg_op_buffer->type == REG_OP_READ_REQ) {
        buffer[3] = 2; //2 bytes for register address and operation bits
        buffer[4] = PAYLOAD_TYPE_RREQ;
    }
    else if(reg_op_buffer->type == REG_OP_WRITE_REQ) {
        buffer[3] = reg_op_buffer->size + 2; //2 bytes for register address and operation bits
        buffer[4] = PAYLOAD_TYPE_WREQ;
        //payload body
        for(uint8_t i=0;i<reg_op_buffer->size;i++) {
            buffer[5+i] = reg_op_buffer->value[reg_op_buffer->size - i - 1];
        }
    }
    buffer[4] |= ((reg_op_buffer->address>>8)&0x0b11111);
    buffer[5] = reg_op_buffer->address&0xFF;
    
    //checksum
    *buffer_len = buffer[3] + 6; // start(2) + source(1) + length(1) + checksum(1) + stop(1)
    buffer[*buffer_len-2] = framework_checksum(buffer, *buffer_len);
    //stop byte
    buffer[*buffer_len-1] = 0xFF;
}
 * */

//UART buffer --> register operation
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

/*
//retrieve the source address of a payload
uint8_t payload_get_source(uint8_t* payload, uint8_t len) {
    return payload[2];
}

//retrieve the register address
uint8_t payload_get_reg_addr(uint8_t* payload, uint8_t len) {
    return ((payload[4]&0b11111)<<8) + payload[5];
}

uint8_t* payload_get_reg_val(uint8_t* payload, uint8_t len) {
    return payload + 6;
}

uint8_t payload_get_reg_size(uint8_t* payload, uint8_t len) {
    return len-8;
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
    payload_build_frame(payload, len*);
}

void payload_build_rreq(uint8_t* payload, uint8_t* len, uint16_t register_address, uint8_t register_size) {
    len* = 8;
    payload[4] = PAYLOAD_TYPE_RREQ | ((register_address>>8)&0b11111);
    payload[5] = (register_address&0xFF);
    payload_build_frame(payload, len*);
}

void payload_build_wres(uint8_t* payload, uint8_t* len, uint16_t register_address) {
    len* = 9;
    payload[4] = PAYLOAD_TYPE_WRES | ((register_address>>8)&0b11111);
    payload[5] = (register_address&0xFF);
    payload[6] = PAYLOAD_FLAG_OK;
    payload_build_frame(payload, len*);
}

void payload_build_rres(uint8_t* payload, uint8_t* len, uint16_t register_address, uint8_t* register_value, uint8_t register_size) {
    len* = 8 + register_size;
    payload[4] = PAYLOAD_TYPE_RRES | ((register_address>>8)&0b11111);
    payload[5] = (register_address&0xFF);
    for(uint8_t i=0;i<register_size;i++) {
        payload[6+i] = register_value[0];
    }
    payload_build_frame(payload, len*);
}*/