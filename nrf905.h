/*
 * File:   nrf905.h
 * Author: Séverin Konishi
 *
 * Created on 15. november 2024, 16:23
 */

#ifndef NRF905_H
#define	NRF905_H
#include <xc.h>

#define NRF905_PACKET_LENGTH   32

void nrf905_setup(uint8_t channel);
uint8_t nrf905_test(void);
void nrf905_send(uint8_t* payload, uint8_t payload_len);
void nrf905_set_channel(uint8_t channel);
void nrf905_receive(uint8_t* payload);

#endif	/* NRF905_H */

