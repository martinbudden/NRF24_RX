#include <NRF24_RX.h>
#include <nrf24_rx_h8_3d.h>
#include <nrf24_rx_cx10.h>

// NRF24L01 pins
static const int CE_PIN = 9;
static const int CSN_PIN = 10;

// uncomment one of the USE_ #defines below to specify the protocol to use
#define USE_CX10
//#define USE_H8_3D

#if defined(USE_CX10)
CX10_RX nrf24(CE_PIN, CSN_PIN);
NRF24_RX *rx = &nrf24;
static const int protocol = NRF24_RX::CX10A;
static const int rcChannelCount = CX10_RX::RC_CHANNEL_COUNT;
#elif defined(USE_H8_3D)
H8_3D_RX nrf24(CE_PIN, CSN_PIN);
NRF24_RX *rx = &nrf24;
static const int protocol = NRF24_RX::H8_3D;
static const int rcChannelCount = H8_3D_RX::RC_CHANNEL_COUNT;
#endif

uint16_t nrf24RcData[rcChannelCount];


void setup()
{
  rx->begin(protocol);

  Serial.begin(57600);
  Serial.println("");
  Serial.print("Receiver Starting, protocol: ");
  Serial.println(String(rx->getProtocolString()));
  Serial.println("");
  delay(100);
}

void printRcData()
{
  Serial.println("");
  Serial.print("rol=");
  Serial.print(nrf24RcData[NRF24_ROLL]);
  Serial.print(",");
  Serial.print("pit=");
  Serial.print(nrf24RcData[NRF24_PITCH]);
  Serial.print(",");
  Serial.print("thr=");
  Serial.print(nrf24RcData[NRF24_THROTTLE]);
  Serial.print(",");
  Serial.print("yaw=");
  Serial.print(nrf24RcData[NRF24_YAW]);
}

void loop()
{
  static bool printedBinding = false;
  const NRF24_RX::received_e dataReceived = rx->dataReceived();
  if (dataReceived == NRF24_RX::RECEIVED_BIND) {
    if (printedBinding == false) {
      Serial.print("Binding");
      printedBinding = true;
    }
    Serial.print(".");
  } else if (dataReceived == NRF24_RX::RECEIVED_DATA) {
    rx->setRcDataFromPayload(nrf24RcData);
    printRcData();
  }
  delay(1);
}

