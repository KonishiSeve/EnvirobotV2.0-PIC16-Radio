/*
 * File:   mapping.h
 * Author: Séverin Konishi
 *
 * Created on 15. november 2024, 16:23
 */

#ifndef MAPPING_H
#define	MAPPING_H
#include <xc.h>

uint8_t mapping_new(uint16_t address_radio, uint16_t address_frame, uint16_t length);
void mapping_get(uint8_t index, uint16_t* address_radio, uint16_t* address_frame, uint16_t* length);
void mapping_set(uint8_t index, uint16_t address_radio, uint16_t address_frame, uint16_t length);


#endif	/* MAPPING_H */

