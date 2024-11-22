/* 
 * File:   utilities.h
 * Author: skonishi
 *
 * Created on 21. novembre 2024, 10:50
 */

#ifndef UTILITIES_H
#define	UTILITIES_H

#include <xc.h>
// ===== LED ===== //
void led_init(void);
void led_state(uint8_t state);

// ===== TIME ===== //
void delay_us(uint32_t delay_us);

#endif	/* UTILITIES_H */

