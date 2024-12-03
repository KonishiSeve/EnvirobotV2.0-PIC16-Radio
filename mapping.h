/* 
 * File:   mapping.h
 * Author: S�verin Konishi
 *
 * Created on 2. d�cember 2024, 14:21
 */

#ifndef MAPPING_H
#define	MAPPING_H
#include <xc.h>

uint8_t mapping_new(uint16_t address_radio, uint16_t address_frame, uint16_t length);
void mapping_get(uint8_t index, uint16_t* address_radio, uint16_t* address_frame, uint16_t* length);
void mapping_set(uint8_t index, uint16_t address_radio, uint16_t address_frame, uint16_t length);


#endif	/* MAPPING_H */

