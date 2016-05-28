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

#pragma once

#include <stdbool.h>
#include <stdint.h>

uint16_t XN297_UnscramblePayload(uint8_t* data, int len, const uint8_t *rxAddr);
uint8_t XN297_WritePayload(uint8_t *data, int len, const uint8_t *rxAddr, NRF24L01* nrf24);

