/* 
 * File:   spi.h
 * Author: skonishi
 *
 * Created on 19. novembre 2024, 14:50
 */

#ifndef SPI_H
#define	SPI_H

#include <xc.h>

void spi_init(void);
void spi_cs(uint8_t state);
uint8_t spi_xfer(uint8_t data);

#endif	/* SPI_H */

