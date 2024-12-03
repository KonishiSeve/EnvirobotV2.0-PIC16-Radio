/* 
 * File:   utilities.h
 * Author: Séverin Konishi
 *
 * Created on 21. november 2024, 10:50
 */

#ifndef UTILITIES_H
#define	UTILITIES_H
#include <xc.h>

// ===== LED ===== //
void led_init(void);
void led_state(uint8_t state);

// ===== TIME ===== //
void delay_us(uint32_t delay_us);
void delay_ms(uint16_t delay_ms);

// ===== REGISTER OPERATIONS ===== //
//max number of bytes for a register (for buffer allocation)
#define REG_MAX_SIZE    8

typedef enum reg_op_types {
    REG_OP_READ_REQ,
    REG_OP_READ_RES,
    REG_OP_WRITE_REQ,
    REG_OP_WRITE_RES
} reg_op_types;

//structure containing the informations of a register operation (request or response to a request)
//used to store the informations while translating a request/response from/to the framework protocol to/from the radio protocol
typedef struct reg_op {
    reg_op_types type;
    uint16_t address;
    uint8_t value[REG_MAX_SIZE];  //LSB first
    uint8_t size;
} reg_op;

#endif	/* UTILITIES_H */

