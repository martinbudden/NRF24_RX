/*
 * This file is part of the Arduino NRF24_RX library.
 *
 * NRF24_RX is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License, <http://www.gnu.org/licenses/>, for
 * more details.
 */

#include <stdbool.h>
#include <stdint.h>
#include "NRF24_RX.h"

NRF24_RX::~NRF24_RX() {}

NRF24_RX::NRF24_RX(NRF24L01* _nrf24)
    : nrf24(_nrf24) {}

const uint8_t *NRF24_RX::payloadPtr(void)
{
    return payload;
}
int NRF24_RX::getChannel(void)
{
    return nrf24->getChannel();
}


