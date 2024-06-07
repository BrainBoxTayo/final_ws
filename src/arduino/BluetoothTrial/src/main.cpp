#include "main.hpp"

#include "BluetoothSerial.h"

// D4:8A:FC:CF:AC:C6 Main ESP
// D4:8A:FC:A8:96:7A BackUP
BluetoothSerial SerialBT;
/*Pin assignments for 1st L298N*/
const unsigned int d1EN_A = 14;
const unsigned int d1IN1_A = 27;
const unsigned int d1IN2_A = 26;

const unsigned int d1IN1_B = 25;
const unsigned int d1IN2_B = 33;
const unsigned int d1EN_B = 32;

/*Pin assignments for 2nd L298N*/
const unsigned int d2EN_A = 23;
const unsigned int d2IN1_A = 22;
const unsigned int d2IN2_A = 1;

const unsigned int d2IN1_B = 3;
const unsigned int d2IN2_B = 21;
const unsigned int d2EN_B = 19;

// Initialize the 4 motors
L298NX2 driver1(d1EN_A, d1IN1_A, d1IN1_B, d1EN_B, d1IN2_B, d1IN2_B);
L298NX2 driver2(d2EN_A, d2IN1_A, d2IN1_B, d2EN_B, d2IN2_B, d2IN2_B);

void setup()
{
  driver1.setSpeed(0);
  driver2.setSpeed(0);
  Serial.begin(115200);
  SerialBT.begin("ESP32test"); // Bluetooth device name
  Serial.println("The device started, now you can pair it with Bluetooth!");
  Serial.setRxBufferSize(SERIAL_SIZE_RX);
}

void loop()
{
  if (SerialBT.available())
  {
    String message = SerialBT.readStringUntil('\n');
    processMessage(message, driver1, driver2);
    //processMessage(message);
  }
  else 
  {
    driver1.stop();
    driver2.stop();
  }
}
