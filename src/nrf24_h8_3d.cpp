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

// This file borrows heavily from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arduino.h>

#include <NRF24.h>
#include "NRF24_RX.h"
#include "nrf24_h8_3d.h"
#include "xn297.h"

/*
 * Deviation transmitter sends 345 bind packets, then starts sending data packets.
 * Packets are send at rate of at least one every 4 milliseconds, ie at least 250Hz.
 * This means binding phase lasts 1.4 seconds, the transmitter then enters the data phase.
 * Other transmitters may vary but should have similar characteristics.
 */

/*
 * H8_3D Protocol
 * No auto acknowledgment
 * Payload size is 20, static
 * Data rate is 1Kbps
 * Bind Phase
 * uses address {0xab,0xac,0xad,0xae,0xaf}, converted by XN297 to {0x41, 0xbd, 0x42, 0xd4, 0xc2}
 * hops between 4 channels
 * Data Phase
 * uses same address as bind phase
 * hops between 4 channels generated from txId received in bind packets
 */


#define FLAG_FLIP       0x01
#define FLAG_RATE_MID   0x02
#define FLAG_RATE_HIGH  0x04
#define FLAG_HEADLESS   0x10 // RTH + headless on H8, headless on JJRC H20
#define FLAG_RTH        0x20 // 360° flip mode on H8 3D, RTH on JJRC H20
#define FLAG_PICTURE    0x40 // on payload[18]
#define FLAG_VIDEO      0x80 // on payload[18]
#define FLAG_CAMERA_UP  0x04 // on payload[18]
#define FLAG_CAMERA_DOWN 0x08 // on payload[18]



#define RF_BIND_CHANNEL_START 0x06
#define RF_BIND_CHANNEL_END 0x26

#define DATA_HOP_TIMEOUT 5000 // 5ms
#define BIND_HOP_TIMEOUT 1000 // 1ms, to find the bind channel as quickly as possible
#define BIND_TIMEOUT 100000 // 20ms

//const uint8_t rxAddr[RX_TX_ADDR_LEN] = {0xc4, 0x57, 0x09, 0x65, 0x21};
const uint8_t H8_3D::rxAddr[H8_3D::RX_ADDR_LEN] = {0x41, 0xbd, 0x42, 0xd4, 0xc2}; // converted XN297 address

H8_3D::~H8_3D() {}

H8_3D::H8_3D(NRF24L01* _nrf24)
    : NRF24_RX(_nrf24)
{
    rfChannels = rfChannelArray;
}

H8_3D::H8_3D(uint8_t _ce_pin, uint8_t _csn_pin)
    : NRF24_RX(_ce_pin, _csn_pin)
{
    rfChannels = rfChannelArray;
}

uint16_t H8_3D::convertToPwm(uint8_t val, int16_t _min, int16_t _max)
{
    int32_t ret = val;
    const int32_t range = _max - _min;
    ret = PWM_RANGE_MIN + ((ret - _min) * PWM_RANGE)/range;
    return (uint16_t)ret;
}

void H8_3D::setRcDataFromPayload(uint16_t *rcData) const
{
    rcData[NRF24_ROLL] = convertToPwm(payload[12], 0xbb, 0x43); // aileron
    rcData[NRF24_PITCH] = convertToPwm(payload[11], 0x43, 0xbb); // elevator
    rcData[NRF24_THROTTLE] = convertToPwm(payload[9], 0, 0xff); // throttle
    const int8_t yawByte = payload[10]; // rudder
    rcData[NRF24_YAW] = yawByte >= 0 ? convertToPwm(yawByte, -0x3c, 0x3c) : convertToPwm(yawByte, 0xbc, 0x44);

    const uint8_t flags = payload[17];
    const uint8_t flags2 = payload[18];
    if (flags & FLAG_RATE_HIGH) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MAX;
    } else if (flags & FLAG_RATE_MID) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIDDLE;
    } else {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIN;
    }

    rcData[RC_CHANNEL_FLIP] = flags & FLAG_FLIP ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_PICTURE] = flags2 & FLAG_PICTURE ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_VIDEO] = flags2 & FLAG_VIDEO ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_HEADLESS] = flags & FLAG_HEADLESS ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_RTH] = flags & FLAG_RTH ? PWM_RANGE_MAX : PWM_RANGE_MIN;

    if (flags2 & FLAG_CAMERA_UP) {
        rcData[NRF24_AUX7] = PWM_RANGE_MAX;
    } else if (flags2 & FLAG_CAMERA_DOWN) {
        rcData[NRF24_AUX7] = PWM_RANGE_MIN;
    } else {
        rcData[NRF24_AUX7] = PWM_RANGE_MIDDLE;
    }

    rcData[NRF24_AUX8] = convertToPwm(payload[14], 0x10, 0x30);
    rcData[NRF24_AUX9] = convertToPwm(payload[15], 0x30, 0x10);
    rcData[NRF24_AUX10] = convertToPwm(payload[16], 0x10, 0x30);
}

void H8_3D::hopToNextChannel(void)
{
    if (protocolState == STATE_DATA) {
        NRF24_RX::hopToNextChannel();
    } else {
        timeOfLastHop = micros();
        ++rfChannelIndex;
        if (rfChannelIndex > RF_BIND_CHANNEL_END) {
            rfChannelIndex = RF_BIND_CHANNEL_START;
        }
        nrf24->setChannel(rfChannelIndex);
    }
}

// The hopping channels are determined by the txId
void H8_3D::setHoppingChannels(void)
{
    // Kludge to allow bugged deviation TX H8_3D implementation to work
    const int bitShift = (protocol == NRF24_RX::H8_3D_DEVIATION) ? 8 : 4;
    rfChannels[0] = 0x06 + ((txId[0]>>bitShift) +(txId[0] & 0x0f)) % 0x0f;
    rfChannels[1] = 0x15 + ((txId[1]>>bitShift) +(txId[1] & 0x0f)) % 0x0f;
    rfChannels[2] = 0x24 + ((txId[2]>>bitShift) +(txId[2] & 0x0f)) % 0x0f;
    rfChannels[3] = 0x33 + ((txId[3]>>bitShift) +(txId[3] & 0x0f)) % 0x0f;
}

bool H8_3D::checkBindPacket(void)
{
    if ((payload[5] == 0x00) && (payload[6] == 0x00) && (payload[7] == 0x01)) {
        if (protocolState == STATE_BIND_0) {
            // do some additional checking to ensure the txId is valid
            const uint32_t checkSum = (payload[1] + payload[2] + payload[3] + payload[4]) & 0xff;
            if (checkSum == payload[8] && payload[0] == 0x13) {
                // payload is all good, so set the txId
                txId[0] = payload[1];
                txId[1] = payload[2];
                txId[2] = payload[3];
                txId[3] = payload[4];
                return true;
            }
        } else {
            // txId has already been set
            return true;
        }
    }
    return false;
}

void H8_3D::setBound(void)
{
    hopTimeout = DATA_HOP_TIMEOUT;
    protocolState = STATE_DATA;
    timeOfLastHop = micros();
    setHoppingChannels();
    rfChannelIndex = 0;
    nrf24->setChannel(rfChannels[0]);
}

bool H8_3D::crcOK(void) const
{
    if (payload[payloadSize] != (payloadCrc >> 8)) {
        return false;
    }
    if (payload[payloadSize + 1] != (payloadCrc & 0xff)) {
        return false;
    }
    return true;
}

bool H8_3D::checkSumOK(void) const
{
    if (payload[0] != 0x13) {
        return false;
    }
    const uint32_t checkSumTxId = (payload[1] + payload[2] + payload[3] + payload[4]) & 0xff;
    if (checkSumTxId != payload[8]) {
        return false;
    }
    uint32_t checkSum = payload[9];
    for (int ii = 10; ii < 19; ++ii) {
        checkSum += payload[ii];
    }
    if ((checkSum & 0xff) != payload[19]) {
        return false;
    }
    return true;
}

/*
 * Returns NRF24L01_RECEIVED_DATA if a data packet was received.
 */
NRF24_RX::received_e H8_3D::dataReceived(void)
{
    NRF24_RX::received_e ret = RECEIVED_NONE;
    // read the payload, processing of payload is deferred
    bool packetReceived = false;
    if (nrf24->readPayloadIfAvailable(payload, payloadSize + CRC_LEN)) {
        payloadCrc = XN297_UnscramblePayload(payload, payloadSize, rxAddr);
        if (crcOK()) {
            timeOfLastPacket = micros();
            packetReceived = true;
        }
    }

    switch (protocolState) {
    case STATE_BIND_0:
        if (packetReceived) {
            if (checkBindPacket()) {
                hopToNextChannel();
                protocolState = STATE_BIND_1;
                payload[0] = 1;
                ret = RECEIVED_BIND;
            }
        } else if (micros() > timeOfLastHop + hopTimeout) {
            hopToNextChannel();
        }
        break;
    case STATE_BIND_1:
        if (packetReceived) {
            if (checkBindPacket()) {
                hopToNextChannel();
                payload[0] = 2;
                ret = RECEIVED_BIND;
            } else {
                // data packet received
                protocolState = STATE_DATA;
                timeOfLastHop = micros();
                setHoppingChannels();
                hopTimeout = DATA_HOP_TIMEOUT;
                rfChannelIndex = 0;
                nrf24->setChannel(rfChannels[0]);
                payload[0] = 3;
                ret = RECEIVED_DATA;
            }
        } else {
            if (micros() > timeOfLastPacket + BIND_TIMEOUT) {
                // transmitter has stopped sending bind packets, so it must be in data mode
                protocolState = STATE_DATA;
                timeOfLastHop = micros();
                setHoppingChannels();
                hopTimeout = DATA_HOP_TIMEOUT;
                rfChannelIndex = 0;
                nrf24->setChannel(rfChannels[0]);
                payload[0] = 4;
                ret = RECEIVED_DATA;
            } else if (micros() > timeOfLastHop + hopTimeout) {
                hopToNextChannel();
            }
        }
        break;
    case STATE_DATA:
        if (packetReceived) {
            ret = RECEIVED_DATA;
            hopToNextChannel();
            payload[0] = 5;
        } else if (micros() > timeOfLastHop + hopTimeout) {
            hopToNextChannel();
        }
        break;
    }
    return ret;
}

void H8_3D::begin(int _protocol, const uint8_t* nrf24_id)
{

    protocol = _protocol;
    protocolState = STATE_BIND_0;
    hopTimeout = BIND_HOP_TIMEOUT;
    rfChannelCount = RF_CHANNEL_COUNT;
    payloadSize = PAYLOAD_SIZE;

    NRF24_RX::initialize(0); // sets PWR_UP, no CRC - hardware CRC not used for XN297

    nrf24->writeReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P1-P5 are left at default values
    nrf24->writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxAddr, RX_ADDR_LEN);
    if ((nrf24_id == 0) || ((nrf24_id[0] | nrf24_id[1] | nrf24_id[2] | nrf24_id[3]) == 0)) {
        rfChannelIndex = RF_BIND_CHANNEL_START;
        nrf24->setChannel(rfChannelIndex);
    } else {
        memcpy(txId, nrf24_id, sizeof(txId));
        setBound();
    }

    nrf24->writeReg(NRF24L01_11_RX_PW_P0, payloadSize + CRC_LEN);  // payload + 2 bytes CRC

    nrf24->setRxMode(); // enter receive mode to start listening for packets
}

