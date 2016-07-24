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

class H8_3D_RX : public NRF24_RX {
private:
    enum {TX_ID_LEN = 4};
    uint8_t txId[TX_ID_LEN]; // transmitter ID, sent in bind packet
    // radio channels for frequency hopping
    enum {RF_CHANNEL_COUNT = 4};
    uint8_t rfChannelArray[RF_CHANNEL_COUNT];
    enum {CRC_LEN = 2};
    enum {PAYLOAD_SIZE = 20};
    enum {STATE_BIND_0 = 0, STATE_BIND_1, STATE_DATA};
    enum {RX_ADDR_LEN = 5};
    static const uint8_t rxAddr[RX_ADDR_LEN];
    uint32_t timeOfLastPacket;
public:
    enum {RC_CHANNEL_COUNT = 14};
private:
    static uint16_t convertToPwm(uint8_t val, int16_t _min, int16_t _max);
    bool checkSumOK(void) const;
protected:
    void setHoppingChannels(const uint8_t *txId);
    void setBound(const uint8_t *txId);
    virtual void hopToNextChannel(void);
    virtual bool checkBindPacket(void);
public:
    virtual ~H8_3D_RX();
    H8_3D_RX(NRF24L01 *nrf24);
    H8_3D_RX(uint8_t _ce_pin, uint8_t _csn_pin);
    virtual void begin(int protocol, const uint32_t *nrf24rx_id = 0);
    virtual void setRcDataFromPayload(uint16_t *rcData) const;
    virtual received_e dataReceived(void);
};

