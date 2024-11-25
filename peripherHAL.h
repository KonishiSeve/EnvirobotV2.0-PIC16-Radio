/* 
 * File:   peripherHAL.h
 * Author: Séverin Konishi
 *
 * Created on 21. novembre 2024, 15:45
 * 
 * This file contains a HAL for the SPI, UART and EEPROM peripherals of the PIC16LF876A
 */

#ifndef PERIPHERHAL_H
#define	PERIPHERHAL_H
#include <xc.h>

// ===== SPI ===== //
void spi_init(void);
void spi_cs(uint8_t state);
uint8_t spi_xfer(uint8_t data);

// ===== UART ===== //
void uart_init(void);
void uart_write(uint8_t data);
void uart_write_buffer(uint8_t* data, uint8_t data_len);
uint8_t uart_read(void);

// ===== EEPROM ===== //
void eeprom_init(void);
void eeprom_write(uint8_t address, uint8_t data);
uint8_t eeprom_read(uint8_t address);

// ===== ADC ====== //
void adc_disable(void);

#endif	/* PERIPHERHAL_H */

