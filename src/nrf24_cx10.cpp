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

// This file borrows heavily from project Deviation,
// see http://deviationtx.com

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <arduino.h>

#include <NRF24.h>
#include "NRF24_RX.h"
#include "nrf24_cx10.h"
#include "xn297.h"

/*
 * Deviation transmitter
 * Bind phase lasts 6 seconds for CX10, for CX10A it lasts until an acknowledgment is received.
 * Other transmitters may vary but should have similar characteristics.
 * For CX10A protocol: after receiving a bind packet, the receiver must send back a data packet with byte[9] = 1 as acknowledgment
 */

/*
 * CX10 Protocol
 * No auto acknowledgment
 * Payload size is 19 and static for CX10A variant, 15 and static for CX10 variant.
 * Data rate is 1Mbps
 * Bind Phase
 * uses address {0xcc, 0xcc, 0xcc, 0xcc, 0xcc}, converted by XN297
 * uses channel 0x02
 * Data phase
 * uses same address as bind phase
 * hops between 4 channels that are set from the txId sent in the bind packet
 */



#define FLAG_FLIP       0x10 // from rudder channel
// flags1
#define FLAG_MODE_MASK  0x03
#define FLAG_HEADLESS   0x04
// flags2
#define FLAG_VIDEO      0x02
#define FLAG_PICTURE    0x04


#define CX10_PROTOCOL_PAYLOAD_SIZE  15
#define CX10A_PROTOCOL_PAYLOAD_SIZE 19
#define ACK_TO_SEND_COUNT 8

const uint8_t CX10::txAddrXN297[RX_ADDR_LEN] = {0x55, 0x0F, 0x71, 0x0C, 0x00}; // converted XN297 address, 0xC710F55 (28 bit)
const uint8_t CX10::rxAddrXN297[RX_ADDR_LEN] = {0x49, 0x26, 0x87, 0x7d, 0x2f}; // converted XN297 address


CX10::CX10(NRF24L01* _nrf24)
    : NRF24_RX(_nrf24) {}

CX10::CX10(uint8_t _ce_pin, uint8_t _csn_pin)
    : NRF24_RX(_ce_pin, _csn_pin) {}

/*
 * Returns true if it is a bind packet.
 */
bool CX10::checkBindPacket(void)
{
    if (payload[0] == 0xaa) { // 10101010
        txId[0] = payload[1];
        txId[1] = payload[2];
        txId[2] = payload[3];
        txId[3] = payload[4];
        return true;
    }
    return false;
}

uint16_t CX10::convertToPwmUnsigned(const uint8_t* pVal)
{
    uint16_t ret = (*(pVal + 1)) & 0x7f; // mask out top bit which is used for a flag for the rudder
    ret = (ret << 8) | *pVal;
    return ret;
}

void CX10::setRcDataFromPayload(uint16_t *rcData) const
{
    const uint8_t offset = (protocol == NRF24_RX::CX10) ? 0 : 4;
    rcData[NRF24_ROLL] = (PWM_RANGE_MAX + PWM_RANGE_MIN) - convertToPwmUnsigned(&payload[5 + offset]);  // aileron
    rcData[NRF24_PITCH] = (PWM_RANGE_MAX + PWM_RANGE_MIN) - convertToPwmUnsigned(&payload[7 + offset]); // elevator
    rcData[NRF24_THROTTLE] = convertToPwmUnsigned(&payload[9 + offset]); // throttle
    rcData[NRF24_YAW] = convertToPwmUnsigned(&payload[11 + offset]);  // rudder

    const uint8_t flags1 = payload[13 + offset];
    const uint8_t rate = flags1 & FLAG_MODE_MASK; // takes values 0, 1, 2
    if (rate == RATE_LOW) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIN;
    } else if (rate == RATE_MID) {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MIDDLE;
    } else {
        rcData[RC_CHANNEL_RATE] = PWM_RANGE_MAX;
    }

    // flip flag is in YAW byte
    rcData[RC_CHANNEL_FLIP] = payload[12 + offset] & FLAG_FLIP ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    const uint8_t flags2 = payload[14 + offset];
    rcData[RC_CHANNEL_PICTURE] = flags2 & FLAG_PICTURE ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_VIDEO] = flags2 & FLAG_VIDEO ? PWM_RANGE_MAX : PWM_RANGE_MIN;
    rcData[RC_CHANNEL_HEADLESS] = flags1 & FLAG_HEADLESS ? PWM_RANGE_MAX : PWM_RANGE_MIN;
}

void CX10::hopToNextChannel(void)
{
    ++rfChannelIndex;
    if (rfChannelIndex >= rfChannelCount) {
        rfChannelIndex = 0;
    }
    nrf24->setChannel(rfChannels[rfChannelIndex]);
}

// The hopping channels are determined by the txId
void CX10::setHoppingChannels(void)
{
    rfChannels[0] = 0x03 + (txId[0] & 0x0F);
    rfChannels[1] = 0x16 + (txId[0] >> 4);
    rfChannels[2] = 0x2D + (txId[1] & 0x0F);
    rfChannels[3] = 0x40 + (txId[1] >> 4);
}

void CX10::setBound(void)
{
    protocolState = STATE_DATA;
    timeOfLastHop = micros();
    setHoppingChannels();
    rfChannelIndex = 0;
    nrf24->setChannel(rfChannels[0]);
}

NRF24_RX::received_e CX10::dataReceived()
{
    static uint8_t ackCount;
    NRF24_RX::received_e ret = RECEIVED_NONE;
    int totalDelayUs;
    uint32_t timeNowUs;

    switch (protocolState) {
    case STATE_BIND:
        if (nrf24->readPayloadIfAvailable(payload, payloadSize + CRC_LEN)) {
            payloadCrc = XN297_UnscramblePayload(payload, payloadSize, rxAddr);
            const bool bindPacket = checkBindPacket();
            if (bindPacket) {
                // set the hopping channels as determined by the txId received in the bind packet
                setHoppingChannels();
                ret = RECEIVED_BIND;
                protocolState = STATE_ACK;
                ackCount = 0;
            }
        }
        break;
    case STATE_ACK:
        // transmit an ACK packet
        ++ackCount;
        totalDelayUs = 0;
        // send out an ACK on the bind channel, required by deviationTx
        payload[9] = 0x01;
        nrf24->setChannel(RF_BIND_CHANNEL);
        nrf24->flushTx();
        XN297_WritePayload(payload, payloadSize, txAddrXN297, nrf24);
        nrf24->setTxMode();// enter transmit mode to send the packet
        // wait for the ACK packet to send before changing channel
        static const int fifoDelayUs = 100;
        while (!(nrf24->readReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_TX_EMPTY))) {
            delayMicroseconds(fifoDelayUs);
            totalDelayUs += fifoDelayUs;
        }
        // send out an ACK on each of the hopping channels, required by CX10 transmitter
        for (uint8_t ii = 0; ii < RF_CHANNEL_COUNT; ++ii) {
            nrf24->setChannel(rfChannels[ii]);
            XN297_WritePayload(payload, payloadSize, txAddrXN297, nrf24);
            nrf24->setTxMode();// enter transmit mode to send the packet
            // wait for the ACK packet to send before changing channel
            while (!(nrf24->readReg(NRF24L01_17_FIFO_STATUS) & BV(NRF24L01_17_FIFO_STATUS_TX_EMPTY))) {
                delayMicroseconds(fifoDelayUs);
                totalDelayUs += fifoDelayUs;
            }
        }
        static const int delayBetweenPacketsUs = 1000;
        if (totalDelayUs < delayBetweenPacketsUs) {
            delayMicroseconds(delayBetweenPacketsUs - totalDelayUs);
        }
        nrf24->setRxMode();// re-enter receive mode after sending ACKs
        if (ackCount > ACK_TO_SEND_COUNT) {
            nrf24->setChannel(rfChannels[0]);
            // and go into data state to wait for first data packet
            protocolState = STATE_DATA;
        }
        break;
    case STATE_DATA:
        timeNowUs = micros();
        // read the payload, processing of payload is deferred
        if (nrf24->readPayloadIfAvailable(payload, payloadSize + CRC_LEN)) {
            XN297_UnscramblePayload(payload, payloadSize, rxAddr);
            hopToNextChannel();
            timeOfLastHop = timeNowUs;
            ret = RECEIVED_DATA;
        }
        if (timeNowUs > timeOfLastHop + hopTimeout) {
            hopToNextChannel();
            timeOfLastHop = timeNowUs;
        }
    }
    return ret;
}

void CX10::begin(int _protocol, const uint8_t* nrf24_id)
{
    protocol = protocol;
    protocolState = STATE_BIND;
    hopTimeout = 5000; // 5ms
    rfChannelCount = RF_CHANNEL_COUNT;
    payloadSize = (protocol == NRF24_RX::CX10) ? CX10_PROTOCOL_PAYLOAD_SIZE : CX10A_PROTOCOL_PAYLOAD_SIZE;

    nrf24->initialize(0); // sets PWR_UP, no CRC - hardware CRC not used for XN297

    nrf24->writeReg(NRF24L01_01_EN_AA, 0); // No auto acknowledgment
    nrf24->writeReg(NRF24L01_02_EN_RXADDR, BV(NRF24L01_02_EN_RXADDR_ERX_P0));  // Enable data pipe 0 only
    nrf24->writeReg(NRF24L01_03_SETUP_AW, NRF24L01_03_SETUP_AW_5BYTES);   // 5-byte RX/TX address
    nrf24->setChannel(RF_BIND_CHANNEL);

    nrf24->writeReg(NRF24L01_06_RF_SETUP, NRF24L01_06_RF_SETUP_RF_DR_1Mbps | NRF24L01_06_RF_SETUP_RF_PWR_n12dbm);
    // RX_ADDR for pipes P2 to P5 are left at default values
    nrf24->flushRx();
    nrf24->writeRegisterMulti(NRF24L01_10_TX_ADDR, txAddrXN297, RX_ADDR_LEN);
    memcpy(rxAddr, rxAddrXN297, RX_ADDR_LEN);
    nrf24->writeRegisterMulti(NRF24L01_0A_RX_ADDR_P0, rxAddr, RX_ADDR_LEN);

    nrf24->writeReg(NRF24L01_08_OBSERVE_TX, 0x00);
    nrf24->writeReg(NRF24L01_11_RX_PW_P0, payloadSize + CRC_LEN); // payload + 2 bytes CRC

    nrf24->writeReg(NRF24L01_1C_DYNPD, 0x00); // Disable dynamic payload length on all pipes

    nrf24->setRxMode(); // enter receive mode to start listening for packets
}

