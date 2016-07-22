#include <NRF24_RX.h>
#include <nrf24_cx10.h>
#include <nrf24_h8_3d.h>
#include <Servo.h>

// NRF24L01 pins
static const int CE_PIN = 9;
static const int CSN_PIN = 10;

// uncomment one of the USE_ #defines below to specify the protocol to use
//#define USE_CX10
#define USE_H8_3D

#if defined(USE_CX10)
CX10 nrf24(CE_PIN, CSN_PIN);
NRF24_RX *rx = &nrf24;
static const int protocol = NRF24_RX::CX10;
static const int rcChannelCount = CX10::RC_CHANNEL_COUNT;
#elif defined(USE_H8_3D)
H8_3D nrf24(CE_PIN, CSN_PIN);
NRF24_RX *rx = &nrf24;
static const int protocol = NRF24_RX::H8_3D_H20;
static const int rcChannelCount = H8_3D::RC_CHANNEL_COUNT;
#endif

uint16_t nrf24RcData[rcChannelCount];


// servo data
static const int SERVO1_PIN = A0; // analog pin 0
static const int SERVO2_PIN = A1; // analog pin 1
Servo servo1; 
Servo servo2; 


void setup() 
{
  rx->begin(protocol);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  Serial.begin(57600);
  Serial.print("Receiver Starting, protocol: ");
  Serial.println(rx->getProtocol());
  Serial.println("");
  delay(100);
}

void loop() 
{
  const NRF24_RX::received_e dataReceived = rx->dataReceived();
  if (dataReceived == NRF24_RX::RECEIVED_DATA) {
    // set the radio control data into the nrf24RcData[] array
    rx->setRcDataFromPayload(nrf24RcData);

    // set servo1 from the throttle value
    int throttle = nrf24RcData[NRF24_THROTTLE];
    // scale throttle to value between 0 and 180 for servo
    int s1 = map(throttle, PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 180);
    servo1.write(s1);

    // set servo2 from the yaw value
    int yaw = nrf24RcData[NRF24_YAW];
    // scale yaw to value between 0 and 180 for servo
    int s2 = map(yaw, PWM_RANGE_MIN, PWM_RANGE_MAX, 0, 180);
    servo2.write(s2);
  }
  delay(1); // delay for 1 millisecond before next request to receiver
}

