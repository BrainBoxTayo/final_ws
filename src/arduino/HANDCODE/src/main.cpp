#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include "main.hpp"

#define LDR_PIN1 4  // thumb
#define LDR_PIN2 36 // index
#define LDR_PIN3 39 // middle
#define LDR_PIN4 34 // ring
#define LDR_PIN5 35 // pinky
#define BUTTON_PIN 18
#define featuresEnabler 0

ros::NodeHandle nh; // This is the ros node handle
std_msgs::Float64 rollMessage;
std_msgs::Float64 pitchMessage;
std_msgs::Float64 thumbMessage;
std_msgs::Float64 indexMessage;
std_msgs::Float64 middleMessage;
std_msgs::Float64 ringMessage;
std_msgs::Float64 pinkyMessage;

// ROS PUBLISHERS
ros::Publisher gyroRoll("/Tayson/hSERV_HAND_position_controller/command", &rollMessage);
ros::Publisher gyroPitch("/Tayson/BASE_FOREARM_position_controller/command", &pitchMessage);
ros::Publisher thumbServo("/Tayson/HAND_thumbbase_position_controller/command", &thumbMessage);
ros::Publisher indexServo("/Tayson/HAND_indexbase_position_controller/command", &indexMessage);
ros::Publisher middleServo("/Tayson/HAND_middlebase_position_controller/command", &middleMessage);
ros::Publisher ringServo("/Tayson/HAND_ringbase_position_controller/command", &ringMessage);
ros::Publisher pinkyServo("/Tayson/HAND_pinkybase_position_controller/command", &pinkyMessage);

// Data Type declarations
Adafruit_MPU6050 mpu;
Accel accelerometer = {0};
Gyro gyroscope = {0};

// calibration values
float pmax = -70;
float pmin = 70; // pitch

float rmax = -45;
float rmin = 45; // roll

int smin = 4096;
int smax = 0; // LDR

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

int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

unsigned int calibration_time = 5000;

// counter for IMUCOUNTER()
int c = 0;

// roll and pitch values
float roll, pitch;

// State variable
int state = 0;

void setup(void)
{
  // Serial.begin(115200);
  // while (!Serial)
  //   delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin())
  { // Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  // Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);

  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Serial.println("");
  // delay(1000);
  // Serial.println("Starting to calculate\n");
  // calculate_IMU_error();
  // Serial.println("Done");
  // delay(1000000);

  // Calibration
  float sensorValue = 0;
  nh.initNode();
  nh.loginfo("\nCALIBRATION STARTING\n");
  nh.loginfo("\n__rotate arm along x axis__\n");
  while (millis() < calibration_time)
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
    if (millis() == 4990 && (rmax == rmin))
    {
      calibration_time += 5000;
      nh.loginfo("\nPLEASE ROTATE ARM!!!\n");
    }
  }
  nh.loginfo("RMAX: ");
  nh.loginfo(String(rmax).c_str());
  nh.loginfo("RMIN: ");
  nh.loginfo(String(rmin).c_str());
  nh.loginfo("\n__rotate arm along y axis__\n");
  calibration_time += 5000; // 10 seconds if calibration went well.
  while (millis() < calibration_time)
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

    if (millis() == 4990 && (pmax == pmin))
    {
      calibration_time += 5000;
      nh.loginfo("\nROTATE ARM PLEASE!!!\n");
    }
  }
  nh.loginfo("PMAX: ");
  nh.loginfo(String(pmax).c_str());
  nh.loginfo("PMIN: ");
  nh.loginfo(String(pmin).c_str());

#if featuresEnabler == 1
  calibration_time += 5000; // 15 seconds if calibration went well.
  nh.loginfo("\n__Close and Open Fingers NOW!!!__\n");
  while (millis() < 15000)
  {
    sensorValue = analogRead(LDR_PIN1);
    if (sensorValue < smin)
    {
      smin = sensorValue;
    }
    if (sensorValue > smax)
    {
      smax = sensorValue;
    }

    if (millis() == 4990 && (smax == smin))
    {
      calibration_time += 5000;
      nh.loginfo("\nFOLLOW INSTRUCTIONS PLEASE!!!\n");
    }
  }
  nh.loginfo("SMAX: ");
  nh.loginfo(String(smax).c_str());
  nh.loginfo("SMIN: ");
  nh.loginfo(String(smin).c_str());
#endif
  // end calibration

  nh.advertise(gyroPitch);
  nh.advertise(gyroRoll);
  nh.advertise(thumbServo);
  nh.advertise(indexServo);
  nh.advertise(middleServo);
  nh.advertise(ringServo);
  nh.advertise(pinkyServo);
}

void loop()
{
  int reading = digitalRead(BUTTON_PIN); // Read the button

  mpuUpdate(mpu, accelerometer, gyroscope);
  float pub_roll = constrain(roll, rmin, rmax); // constrains roll to min and max
  pub_roll = map(roll, rmin, rmax, 0, 3.14);    // map to radians
  pub_roll -= 1.57;
  float pub_pitch = constrain(pitch, pmin, pmax); // constrains pitch
  pub_pitch = map(pitch, pmin, pmax, 0, 3.14);    // map to radians
  pub_pitch -= 1.57;

  pitchMessage.data = pub_pitch;
  rollMessage.data = pub_roll;
  thumbMessage.data = ldrUpdate(LDR_PIN1);
  indexMessage.data = ldrUpdate(LDR_PIN2);
  middleMessage.data = ldrUpdate(LDR_PIN3);
  ringMessage.data = ldrUpdate(LDR_PIN4);
  pinkyMessage.data = ldrUpdate(LDR_PIN5);

  // if the switch changes
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }
  if (millis() - lastDebounceTime > debounceDelay)
  {
    if (reading != buttonState)
    {
      buttonState = reading;

      if (buttonState == HIGH)
      {

        if (state == 3)
        {
          nh.loginfo("STATE RESET!!");
          state = 0;
        }
        else
        {
          nh.loginfo("STATE CHANGE!!");
          state += 1;
        }
      }
    }
  }
  // STATE MACHINE
  switch (state)
  {
  case 0:
    // fingers only
    thumbServo.publish(&thumbMessage);
    indexServo.publish(&indexMessage);
    middleServo.publish(&middleMessage);
    ringServo.publish(&ringMessage);
    pinkyServo.publish(&pinkyMessage);
    break;

  case 1:
    // arm only
    gyroPitch.publish(&pitchMessage);
    break;

  case 2:
    // wrist only
    gyroRoll.publish(&rollMessage);
    break;

  case 3:
    // ALL
    gyroRoll.publish(&rollMessage);
    gyroPitch.publish(&pitchMessage);
    thumbServo.publish(&thumbMessage);
    indexServo.publish(&indexMessage);
    middleServo.publish(&middleMessage);
    ringServo.publish(&ringMessage);
    pinkyServo.publish(&pinkyMessage);
    break;

  default:
    break;
  }
  lastButtonState = reading;

  nh.spinOnce();
}

/*
 * ldrUpdate - updates the ldr resistance values
 * @pin: Pin the ldr is connected to on the ESP32
 * Return: sensorValue (LDR reading)
 */
int ldrUpdate(int pin)
{
  int sensorValue = analogRead(pin);
  sensorValue = constrain(sensorValue, smin, smax);
  sensorValue = map(sensorValue, smin, smax, 0, 180);

  return sensorValue;
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