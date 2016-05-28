/*
 * This file is part of the Arduino NRF24_RX library.
 *
 * Written by Martin Budden
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License, <http://www.gnu.org/licenses/>, for
 * more details.
 *
 * All text above and this condition must be included in any redistribution.
 */

#include <stdbool.h>
#include <stdint.h>
#include "NRF24_RX.h"

NRF24_RX::~NRF24_RX() {}

NRF24_RX::NRF24_RX(NRF24L01* _nrf24)
    : nrf24(_nrf24) {}

NRF24_RX::NRF24_RX(uint8_t ce_pin, uint8_t csn_pin)
{
    static NRF24L01 nrf24L01(ce_pin, csn_pin);
    nrf24 = &nrf24L01;
}

void NRF24_RX::initialize(uint8_t baseConfig)
{
    nrf24->initialize(baseConfig);

    nrf24->writeReg(NRF24L01_01_EN_AA, 0); // No auto acknowledgment
    nrf24->writeReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));
    nrf24->writeReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    nrf24->writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxTxAddr, RX_TX_ADDR_LEN);
    nrf24->writeReg(NRF24L01_08_OBSERVE_TX, 0x00);
    nrf24->writeReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes
    nrf24->writeReg(NRF24L01_11_RX_PW_P0, payloadSize);
}

