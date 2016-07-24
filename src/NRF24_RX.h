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
#include <NRF24.h>


#define PWM_RANGE_MIN 1000
#define PWM_RANGE_MAX 2000
#define PWM_RANGE_MIDDLE 1500
#define PWM_RANGE (PWM_RANGE_MAX - PWM_RANGE_MIN)

#define MAX_SUPPORTED_RC_CHANNEL_COUNT 18

// RC channels in AETR order
typedef enum {
    NRF24_ROLL = 0,
    NRF24_PITCH,
    NRF24_THROTTLE,
    NRF24_YAW,
    NRF24_AUX1,
    NRF24_AUX2,
    NRF24_AUX3,
    NRF24_AUX4,
    NRF24_AUX5,
    NRF24_AUX6,
    NRF24_AUX7,
    NRF24_AUX8,
    NRF24_AUX9,
    NRF24_AUX10,
    NRF24_AUX11,
    NRF24_AUX12,
    NRF24_AUX13,
    NRF24_AUX14
} nrf24_AETR_t;

// RC channels as used by deviation
#define RC_CHANNEL_RATE        NRF24_AUX1
#define RC_CHANNEL_FLIP        NRF24_AUX2
#define RC_CHANNEL_PICTURE     NRF24_AUX3
#define RC_CHANNEL_VIDEO       NRF24_AUX4
#define RC_CHANNEL_HEADLESS    NRF24_AUX5
#define RC_CHANNEL_RTH         NRF24_AUX6

class NRF24_RX {
public:
    typedef enum {
        V202_250K = 0,
        V202_1M,
        SYMA_X,
        SYMA_X5C,
        CX10,
        CX10A,
        H8_3D,
        PROTOCOL_COUNT
    } protocol_e;

    typedef enum {
        RECEIVED_NONE = 0,
        RECEIVED_BIND = 1,
        RECEIVED_DATA = 2
    } received_e;
    static const char *protocolString[PROTOCOL_COUNT];
protected:
    NRF24L01* nrf24;
    uint8_t protocol;
    uint8_t protocolState;
    uint8_t rfChannelCount;
    uint8_t rfChannelIndex;
    uint8_t *rfChannels;
    uint32_t hopTimeout;
    uint32_t timeOfLastHop;
    uint8_t payload[NRF24L01_MAX_PAYLOAD_SIZE+32];
    uint16_t payloadCrc;
    uint8_t payloadSize;
protected:
    bool crcOK(uint16_t crc) const;
    virtual void hopToNextChannel(void);
    virtual bool checkBindPacket(void) = 0;
public:
    virtual ~NRF24_RX();
    NRF24_RX(NRF24L01 *_nrf24);
    NRF24_RX(uint8_t ce_pin, uint8_t csn_pin);
    void initialize(uint8_t baseConfig, uint8_t rfDataRate);
    virtual void begin(int protocol, const uint32_t *nrf24_id = 0) = 0;
    virtual void setRcDataFromPayload(uint16_t *rcData) const = 0;
    virtual received_e dataReceived(void) = 0;
// debugging and instrumentation functions
    const uint8_t *payloadPtr(void) const {return payload;}
    int getPayloadSize(void) const {return payloadSize;}
    uint16_t getPayloadCrc(void) const {return payloadCrc;}
    int getChannel(void) const {return nrf24->getChannel();}
    uint8_t getProtocol(void) const {return protocol;}
    const char *getProtocolString(void) const {return protocolString[protocol];}
};

