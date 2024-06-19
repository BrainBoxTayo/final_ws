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

typedef enum {
    FINGER,
    ARM,
    HAND,
    ALL
} STATE;

void mpuUpdate(Adafruit_MPU6050 &mpu, Accel &accel, Gyro &gyro);
float ldrUpdate(int pin);
void calculate_IMU_error();
#endif