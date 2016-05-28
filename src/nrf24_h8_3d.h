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

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "NRF24_RX.h"

class H8_3D : public NRF24_RX {
private:
    enum {TX_ID_LEN = 4};
    uint8_t txId[TX_ID_LEN];
    // radio channels for frequency hopping
    enum {RF_CHANNEL_COUNT = 4};
    uint8_t rfChannels[RF_CHANNEL_COUNT];
    enum {STATE_BIND = 0, STATE_DATA};
    enum {CRC_LEN = 2};
private:
    static uint16_t convertToPwm(uint8_t val, int16_t _min, int16_t _max);
    bool crcOK(uint16_t crc) const;
    bool checkSumOK(void) const;
protected:
    virtual void setHoppingChannels(void);
    virtual void setBound(void);
    virtual void hopToNextChannel(void);
    virtual bool checkBindPacket(void);
public:
    virtual ~H8_3D();
    H8_3D(NRF24L01* nrf24);
    H8_3D(uint8_t _ce_pin, uint8_t _csn_pin);
    virtual void begin(int protocol, const uint8_t* nrf24_id);
    virtual void setRcDataFromPayload(uint16_t *rcData) const;
    virtual received_e dataReceived(void);
};

