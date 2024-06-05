#ifndef MAIN_HPP
#define MAIN_HPP

#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Wire.h>

typedef struct accel
{
    float AccX;
    float AccY;
    float AccZ;
} Accel;
typedef struct gyro
{
    float GyroX;
    float GyroY;
    float GyroZ;
} Gyro;

// deprecated lol
// enum state {
//     FINGER_MOVE,
//     ARM_MOVE,
//     HAND_ROTATE,
//     ALL_MOVE
// };

void mpuUpdate(Adafruit_MPU6050 &mpu, Accel &accel, Gyro &gyro);
int ldrUpdate(int pin);
void calculate_IMU_error();
#endif