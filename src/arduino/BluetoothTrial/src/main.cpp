#include "main.hpp"
#include "BluetoothSerial.h"

// D4:8A:FC:CF:AC:C6 Main ESP
// D4:8A:FC:A8:96:7A BackUP
BluetoothSerial SerialBT;

/*Pin assignments for 1st L298N*/
const unsigned int d1EN_A = 15;
const unsigned int d1IN1_A = 2;
const unsigned int d1IN2_A = 0;

const unsigned int d1IN1_B = 4;
const unsigned int d1IN2_B = 5;
const unsigned int d1EN_B = 18;

/*Pin assignments for 2nd L298N*/
const unsigned int d2EN_A = 14;
const unsigned int d2IN1_A = 27;
const unsigned int d2IN2_A = 26;

const unsigned int d2IN1_B = 25;
const unsigned int d2IN2_B = 33;
const unsigned int d2EN_B = 32;

// Initialize the 4 motors
L298NX2 driver1(d1EN_A, d1IN1_A, d1IN1_B, d1EN_B, d1IN2_B, d1IN2_B);
L298NX2 driver2(d2EN_A, d2IN1_A, d2IN1_B, d2EN_B, d2IN2_B, d2IN2_B);

// Initialize Servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

String receivedData = ""; // Buffer to accumulate incoming data

unsigned long lastMessageTime = 0;          // Time when the last message was received
const unsigned long timeoutDuration = 1000; // Timeout duration in milliseconds

void callback_ESP_BT_data(const uint8_t *buffer, size_t size)
{
  for (size_t i = 0; i < size; i++)
  {
    char c = (char)buffer[i];
    if (c == '\n')
    { // End of message
      processMessage((const unsigned char *)receivedData.c_str(), driver1, driver2, pwm);
      receivedData = "";          // Clear buffer for next message
      lastMessageTime = millis(); // Update the last message time
    }
    else
    {
      receivedData += c; // Accumulate data
    }
  }
}

void setup()
{
  driver1.setSpeed(0);
  driver2.setSpeed(0);
#if SERIAL_ENABLE == 1
  Serial.setRxBufferSize(SERIAL_SIZE_RX);
  Serial.begin(115200);
  Serial.println("The device started, now you can pair it with Bluetooth!");
#endif
  SerialBT.begin("ESP32test"); // Bluetooth device name
  SerialBT.onData(callback_ESP_BT_data);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  pwm.setPWM(0, 0, 480);
  pwm.setPWM(1, 0, 100);
  pwm.setPWM(2, 0, 480);
  pwm.setPWM(3, 0, 480);
  pwm.setPWM(4, 0, 480);
  pwm.setPWM(5, 0, SERVOMIN);
  pwm.setPWM(6, 0, SERVOMIN);
  pwm.setPWM(7, 0, SERVOMIN);
  delay(10);
}

void loop()
{
  unsigned long currentTime = millis();

  // Check if the timeout duration has passed without receiving new data
  if (currentTime - lastMessageTime > timeoutDuration)
  {
    driver1.stop();
    driver2.stop();
  }
}
