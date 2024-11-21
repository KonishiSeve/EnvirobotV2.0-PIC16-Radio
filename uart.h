/* 
 * File:   uart.h
 * Author: skonishi
 *
 * Created on 19. novembre 2024, 14:49
 */

#ifndef UART_H
#define	UART_H

#include <xc.h>

void uart_init(void);
void uart_write(uint8_t data);

#endif	/* UART_H */

