/* 
 * File:   nrf905.h
 * Author: skonishi
 *
 * Created on 20. novembre 2024, 10:07
 */

#ifndef NRF905_H
#define	NRF905_H
#include <xc.h>

#define NRF905_PACKET_LENGTH   32

// state of each pin for each power mode: PWR_UP | TRX_CE | TX_EN
#define NRF905_MODE_DOWN        (0b000)
#define NRF905_MODE_STANDBY_RX  (0b100)
#define NRF905_MODE_STANDBY_TX  (0b101)
#define NRF905_MODE_RX          (0b110)
#define NRF905_MODE_TX          (0b111)

void nrf905_setup(void);
uint8_t nrf905_test(void);
void nrf905_send(uint8_t* payload, uint8_t payload_len);
void nrf905_set_mode(uint8_t mode);
void nrf905_receive(uint8_t* payload);

#endif	/* NRF905_H */

