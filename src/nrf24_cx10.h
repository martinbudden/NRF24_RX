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
 * All the above text and this condition must be included in any redistribution.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "NRF24_RX.h"

class CX10 : public NRF24_RX {
private:
    enum {TX_ID_LEN = 4};
    uint8_t txId[TX_ID_LEN]; // transmitter ID, sent in bind packet
    // radio channels for frequency hopping
    enum {RF_BIND_CHANNEL = 2};
    enum {RF_CHANNEL_COUNT = 4};
    uint8_t rfChannelArray[RF_CHANNEL_COUNT];
    enum {STATE_BIND = 0, STATE_ACK = 1, STATE_DATA = 2};
    enum {CRC_LEN = 2};
    enum {RATE_LOW = 0, RATE_MID = 1, RATE_HIGH = 2};
    enum {RX_ADDR_LEN = 5};
    static const uint8_t rxAddr[RX_ADDR_LEN];
    static const uint8_t txAddr[RX_ADDR_LEN];
public:
    enum {RC_CHANNEL_COUNT = 9};
private:
    static uint16_t convertToPwmUnsigned(const uint8_t *pVal);
protected:
    virtual void setHoppingChannels(void);
    virtual bool checkBindPacket(void);
    virtual void setBound(void);
public:
    virtual ~CX10();
    CX10(NRF24L01 *nrf24);
    CX10(uint8_t _ce_pin, uint8_t _csn_pin);
    virtual void begin(int protocol, const uint8_t *nrf24_id = 0);
    virtual void setRcDataFromPayload(uint16_t *rcData) const;
    virtual received_e dataReceived(void);
};

