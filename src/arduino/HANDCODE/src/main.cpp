#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include "main.hpp"

ros::NodeHandle nh; // This is the ros node handle
std_msgs::Float64 rollMessage;
std_msgs::Float64 pitchMessage;

//ROS PUBLISHERS
ros::Publisher gyroRoll("/Tayson/hSERV_HAND_position_controller/command", &rollMessage);
ros::Publisher gyroPitch("/Tayson/BASE_FOREARM_position_controller/command", &pitchMessage);
// Data Type declarations
Adafruit_MPU6050 mpu;
Accel accelerometer = {0};
Gyro gyroscope = {0};

// calibration values
float pmax = -90;
float pmin = 90;

float rmax = -45;
float rmin = 45;

// global variables
float previousTime;
float currentTime;
float elapsedTime;

float gyroAngleX = 0;
float gyroAngleY = 0;

float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleZ;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;

//counter for IMUCOUNTER()
int c = 0;

// roll and pitch values
float roll, pitch;

void setup(void)
{
  // Serial.begin(115200);
  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  {// Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  // Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Serial.println("");
  // delay(1000);
  // Serial.println("Starting to calculate\n");
  // calculate_IMU_error();
  // Serial.println("DOne");
  // delay(1000000);

  // Calibration
  float sensorValue = 0;
  while (millis() < 5000)
  {
    mpuUpdate(mpu, accelerometer, gyroscope);
    sensorValue = roll;

    if (sensorValue > rmax)
    {
      rmax = sensorValue;
    }
    if (sensorValue < rmin)
    {
      rmin = sensorValue;
    }
  }

   while (millis() < 10000)
  {
    mpuUpdate(mpu, accelerometer, gyroscope);
    sensorValue = pitch;

    if (sensorValue > pmax)
    {
      pmax = sensorValue;
    }
    if (sensorValue < pmin)
    {
      pmin = sensorValue;
    }
  }

  nh.initNode();
  nh.advertise(gyroPitch);
  nh.advertise(gyroRoll);
}

void loop()
{
  mpuUpdate(mpu, accelerometer, gyroscope);
  float pub_roll = constrain(roll, rmin, rmax); // constrains roll to min and max
  pub_roll = map(roll, rmin, rmax, -1.57, 1.57); //map to radians
  float pub_pitch = constrain(pitch, pmin, pmax); //constrains pitch
  pub_pitch = map(pitch, pmax, pmin, 1.57, -1.57); //map to radians
  rollMessage.data = pub_roll;
  gyroRoll.publish(&rollMessage);
  pitchMessage.data = pitch;
  gyroPitch.publish(&pitchMessage);
  nh.spinOnce();
  delay(10);
} 

/*
* mpuUpdate - Updates the roll and pitch values from the mpu
* @mpu: mpu object
* @accel: accel struct, stores xyz accelerometer readings.
* @gyro: gyro struct, stores xyz gyroscope readings
*/

void mpuUpdate(Adafruit_MPU6050 &mpu, Accel &accel, Gyro &gyro)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro.GyroX = g.gyro.x;
  gyro.GyroY = g.gyro.y;

  accel.AccX = a.acceleration.x;
  accel.AccY = a.acceleration.y;
  accel.AccZ = a.acceleration.z;

  accAngleX = (atan(accel.AccY / sqrt(pow(accel.AccX, 2) + pow(accel.AccY, 2))) * 180 / PI);
  accAngleY = (atan(-1 * accel.AccX / sqrt(pow(accel.AccY, 2) + pow(accel.AccY, 2))) * 180 / PI);

  currentTime = millis();
  previousTime = currentTime;
  elapsedTime = (currentTime - previousTime) / 1000; // in seconds

  // Convert raw values to deg/s
  gyroAngleX = gyroAngleX + gyro.GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + gyro.GyroY * elapsedTime;

  // Complementary Filter
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  gyroAngleX = roll;
  gyroAngleY = pitch;

  // Serial.print(roll);
  // Serial.print("/");
  // Serial.println(pitch);
}
/*
* calculate_IMU_error - calculates the error values for the accelerometer and gyro data. ONLY RAN ONCE!!!!
*/
void calculate_IMU_error()
{
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  while (c < 200)
  {
    AccX = a.acceleration.x;
    AccY = a.acceleration.y;
    AccZ = a.acceleration.z;
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI)) - 10;
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI)) + 1;
    c++;
  }
  // Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200)
  {

    GyroX = g.gyro.x;
    GyroY = g.gyro.y;
    GyroZ = g.gyro.z;
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  // Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}