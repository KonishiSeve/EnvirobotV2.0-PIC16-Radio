/* 
 * File:   protocols.h
 * Author: skonishi
 *
 * Created on 21. novembre 2024, 16:51
 */

#ifndef PROTOCOLS_H
#define	PROTOCOLS_H

#include <xc.h>

#define PIC_ADDRESS 0xFE

uint8_t payload_verify(uint8_t* payload, uint8_t len);

#endif	/* PROTOCOLS_H */

