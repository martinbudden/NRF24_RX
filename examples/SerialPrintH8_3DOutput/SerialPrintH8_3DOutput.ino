#include <NRF24_RX.h>
#include <nrf24_h8_3d.h>

#define CE_PIN   9
#define CSN_PIN 10
NRF24L01 nrf24(CE_PIN, CSN_PIN);
H8_3D h8_3d(&nrf24);
NRF24_RX *nrf24_rx = &h8_3d;

uint16_t nrf24RcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];

void setup()
{
  Serial.begin(57600);
  delay(500);

  nrf24_rx->begin(NRF24_RX::H8_3D, 0);
  Serial.println("");
  Serial.print("Receiver Starting, protocol: ");
  Serial.println(nrf24_rx->protocolName());
  Serial.println("");
  delay(100);
}

void printRcData()
{
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
  Serial.println(nrf24RcData[NRF24_YAW]);
}

void loop()
{
  static bool printedBinding = false;
  const NRF24_RX::received_e dataReceived = nrf24_rx->dataReceived();
  if (dataReceived == NRF24_RX::RECEIVED_BIND) {
    if (printedBinding == false) {
      Serial.print("Binding");
      printedBinding = true;
    }
    Serial.print(".");
  } else if (dataReceived == NRF24_RX::RECEIVED_DATA) {
    nrf24_rx->setRcDataFromPayload(nrf24RcData);
    printRcData();
  }
  delay(1);
}

