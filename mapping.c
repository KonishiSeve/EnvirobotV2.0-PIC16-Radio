#include "mapping.h"
#include "peripherHAL.h"
#define EEPROM_ADDR_MAP_NB      0x01
#define EEPROM_ADDR_MAP_BASE    0x10

uint8_t mapping_new(uint16_t address_radio, uint16_t address_frame, uint16_t length) {
    uint8_t mappings_nb = eeprom_read(EEPROM_ADDR_MAP_NB);
    mapping_set(mappings_nb++, address_radio, address_frame, length);
    eeprom_write(EEPROM_ADDR_MAP_NB, mappings_nb);
    return mappings_nb;
}

void mapping_get(uint8_t index, uint16_t* address_radio, uint16_t* address_frame, uint16_t* length) {
    uint8_t eeprom_readings[4];
    //read a mapping from the EEPROM
    eeprom_read_buffer(eeprom_readings, EEPROM_ADDR_MAP_BASE+index*4, 4);
    //decode the mapping
    //l:length bits, r:radio address bits, f:framework address bits -> llllllrr rrrrrrrr lllfffff ffffffff
    *address_radio = (uint16_t)((eeprom_readings[0]&0b11)<<8) | (uint16_t)(eeprom_readings[1]);
    *length = (uint16_t)((eeprom_readings[0]&0b11111100)<<1) | (uint16_t)((eeprom_readings[2])>>5);
    *address_frame = (uint16_t)((eeprom_readings[2]&0b11111)<<8) | (uint16_t)(eeprom_readings[3]);
    return;
}

void mapping_set(uint8_t index, uint16_t address_radio, uint16_t address_frame, uint16_t length) {
    //l:length bits, r:radio address bits, f:framework address bits -> llllllrr rrrrrrrr lllfffff ffffffff
    eeprom_write(EEPROM_ADDR_MAP_BASE + index*4, ((length>>1)&0b11111100) | ((address_radio>>8)&0b00000011));
    eeprom_write(EEPROM_ADDR_MAP_BASE + index*4 + 1, address_radio&0xFF);
    eeprom_write(EEPROM_ADDR_MAP_BASE + index*4 + 2, ((length<<5)&0b11100000) | ((address_frame>>8)&0b00011111));
    eeprom_write(EEPROM_ADDR_MAP_BASE + index*4 + 3, address_frame&0xFF);
    return;
}