/* 
 * File:   protocols.h
 * Author: Séverin Konishi
 *
 * Created on 21. november 2024, 16:51
 */

#ifndef PROTOCOLS_H
#define	PROTOCOLS_H

#include <xc.h>
#include "utilities.h"

//Source address of the Radio PIC when sending framework packets
#define PIC_ADDRESS 0xFE

uint8_t framework_checksum(uint8_t* buffer, uint8_t buffer_len);
uint8_t framework_verify(uint8_t* buffer, uint8_t buffer_len);
void framework_encode(uint8_t* buffer, uint8_t* buffer_len, reg_op* reg_op_buffer);
void framework_decode(uint8_t* buffer, uint8_t buffer_len, reg_op* reg_op_buffer);
void framework_send(reg_op* reg_op_buffer);

#endif	/* PROTOCOLS_H */

