/* 
 * File:   protocols.h
 * Author: skonishi
 *
 * Created on 21. novembre 2024, 16:51
 */

#ifndef PROTOCOLS_H
#define	PROTOCOLS_H

#include <xc.h>
#include "utilities.h"

#define PIC_ADDRESS 0xFE

uint8_t framework_checksum(uint8_t* buffer, uint8_t buffer_len);
uint8_t framework_verify(uint8_t* buffer, uint8_t buffer_len);
void framework_encode(uint8_t* buffer, uint8_t* buffer_len, reg_op* reg_op_buffer);
void framework_decode(uint8_t* buffer, uint8_t buffer_len, reg_op* reg_op_buffer);
void framework_send(reg_op* reg_op_buffer);
/*
void framework_encode(uint8_t* buffer, uint8_t buffer_len, reg_op reg_op_buffer);

uint8_t payload_verify(uint8_t* payload, uint8_t len);
void payload_build_wreq(uint8_t* payload, uint8_t* len, uint16_t register_address, uint8_t* register_value, uint8_t register_size);
void payload_build_rreq(uint8_t* payload, uint8_t* len, uint16_t register_address, uint8_t register_size);
 * */
#endif	/* PROTOCOLS_H */

