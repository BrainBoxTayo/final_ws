#ifndef MAIN_H
#define MAIN_H
#include <Arduino.h>
#include <WString.h>
#include <L298NX2.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>


#define MAX_SPEED_WHEEL 21
#define MIN_SPEED_WHEEL 0
#define LOW_PWM 100
#define HIGH_PWM 255
#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#define SERIAL_SIZE_RX 2048 //should fix the overflow error
#define MAX_COMMANDS 10
#define SERIAL_ENABLE 1
void processMessage(String message);
void processMessage(const unsigned char *buffer, L298NX2 &firstBoard, L298NX2 &secondBoard, Adafruit_PWMServoDriver &pwm);

#endif /*MAIN_H*/