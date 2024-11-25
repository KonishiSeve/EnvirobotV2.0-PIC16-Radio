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


// ===== REGISTER ===== //
typedef enum reg_op_types {
    REG_OP_READ_REQ,
    REG_OP_READ_RES,
    REG_OP_WRITE_REQ,
    REG_OP_WRITE_RES
} reg_op_types;

typedef struct reg_op {
    reg_op_types type;
    uint16_t address;
    uint8_t value[32];  //LSB first
    uint8_t size;
} reg_op;

#endif	/* UTILITIES_H */

