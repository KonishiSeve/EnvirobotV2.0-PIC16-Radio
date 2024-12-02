/* 
 * File:   radio_protocol.h
 * Author: Séverin Konishi
 *
 * Created on 25. november 2024, 11:45
 */

#ifndef RADIO_PROTOCOL_H
#define	RADIO_PROTOCOL_H
#include <xc.h>
#include "utilities.h"

void radio_decode(uint8_t* buffer, uint8_t size, reg_op* reg_op_buffer);
void radio_send(reg_op* reg_op_buffer);
#endif	/* RADIO_PROTOCOL_H */