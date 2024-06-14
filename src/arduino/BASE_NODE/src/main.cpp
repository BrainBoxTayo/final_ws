#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>

#define MAX_SPEED_WHEEL 21
#define MIN_SPEED_WHEEL 0
#define LOW_PWM 100
#define HIGH_PWM 255
#define SERVOMIN 150  // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525  // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

BluetoothSerial BTSerial;
// Initialize Servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/*Pin assignments for 1st L298N*/
const int d1EN_A = 15;
const int d1IN1_A = 2;
const int d1IN2_A = 0;

const int d1IN1_B = 4;
const int d1IN2_B = 5;
const int d1EN_B = 18;

/*Pin assignments for 2nd L298N*/
const int d2EN_A = 13;
const int d2IN1_A = 12;
const int d2IN2_A = 14;

const int d2IN1_B = 27;
const int d2IN2_B = 26;
const int d2EN_B = 25;

ros::NodeHandle nh;

std_msgs::Float32 frontWheelLeft_msg;
std_msgs::Float32 frontWheelRight_msg;
std_msgs::Float32 rearWheelLeft_msg;
std_msgs::Float32 rearWheelRight_msg;
std_msgs::Float64 indexFinger_msg;
std_msgs::Float64 middleFinger_msg;
std_msgs::Float64 ringFinger_msg;
std_msgs::Float64 pinkyFinger_msg;
std_msgs::Float64 thumbFinger_msg;
std_msgs::Float64 arm_msg;
std_msgs::Float64 wrist_msg;

// Helper Functions
void Forward(const int pin1, const int pin2)
{
  digitalWrite(pin1, HIGH);
  digitalWrite(pin2, LOW);
}
void Backward(const int pin1, const int pin2)
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
}
void Stop(const int pin1, const int pin2)
{
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, LOW);
}

/*CALLBACK FUNCTIONS*/
void frontLeftCb(const std_msgs::Float32 &frontWheelLeft_msg)
{
  int speed = constrain(abs(frontWheelLeft_msg.data), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
  int pwmValue = map(speed, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM);
  if (frontWheelLeft_msg.data > 0)
  {
    Forward(d1IN1_B, d1IN2_B);
    analogWrite(d1EN_B, pwmValue);
  }
  else if (frontWheelLeft_msg.data < 0)
  {
    Backward(d1IN1_B, d1IN2_B);
    analogWrite(d2EN_B, pwmValue);
  }
  else
  {
    Stop(d1IN1_B, d1IN2_B);
    analogWrite(d2EN_B, pwmValue);
  }
}
void frontRightCb(const std_msgs::Float32 &frontWheelRight_msg)
{
  int speed = constrain(abs(frontWheelRight_msg.data), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
  int pwmValue = map(speed, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM);
  if (frontWheelRight_msg.data > 0)
  {
    Forward(d1IN1_A, d1IN2_A);
    analogWrite(d1EN_A, pwmValue);
  }
  else if (frontWheelRight_msg.data < 0)
  {
    Backward(d1IN1_A, d1IN2_A);
    analogWrite(d1EN_A, pwmValue);
  }
  else
  {
    Stop(d1IN1_A, d1IN2_A);
    analogWrite(d1EN_A, pwmValue);
  }
}
void rearLeftCb(const std_msgs::Float32 &rearWheelLeft_msg)
{
  int speed = constrain(abs(rearWheelLeft_msg.data), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
  int pwmValue = map(speed, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM);
  if (rearWheelLeft_msg.data > 0)
  {
    Forward(d2IN1_A, d2IN2_A);
    analogWrite(d2EN_A, pwmValue);
  }
  else if (rearWheelLeft_msg.data < 0)
  {
    Backward(d2IN1_A, d2IN2_A);
    analogWrite(d2EN_A, pwmValue);
  }
  else
  {
    Stop(d2IN1_A, d2IN2_A);
    analogWrite(d2EN_A, pwmValue);
  }
}
void rearRightCb(const std_msgs::Float32 &rearwheelRight_msg)
{
  int speed = constrain(abs(rearWheelRight_msg.data), MIN_SPEED_WHEEL, MAX_SPEED_WHEEL);
  int pwmValue = map(speed, MIN_SPEED_WHEEL, MAX_SPEED_WHEEL, LOW_PWM, HIGH_PWM);
  if (rearwheelRight_msg.data > 0)
  {
    Forward(d2IN1_B, d2IN2_B);
    analogWrite(d2EN_B, pwmValue);
  }
  else if (rearwheelRight_msg.data < 0)
  {
    Backward(d2IN1_B, d2IN2_B);
    analogWrite(d2EN_B, pwmValue);
  }
  else
  {
    Stop(d2IN1_B, d2IN2_B);
    analogWrite(d2EN_B, pwmValue);
  }
}

void thumbCallback(const std_msgs::Float64 &thumbFinger_msg)
{
  int angle = map((abs(thumbFinger_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, 480, 150);
  pwm.setPWM(0, 0, position);
}

void indexCallback(const std_msgs::Float64 &indexFinger_msg)
{
  int angle = map((abs(indexFinger_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, 100, 450);
  pwm.setPWM(1, 0, position);
}

void middleCallback(const std_msgs::Float64 &middleFinger_msg)
{
  int angle = map((abs(middleFinger_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, 480, 100);
  pwm.setPWM(2, 0, position);
}

void ringCallback(const std_msgs::Float64 &ringFinger_msg)
{
  int angle = map((abs(ringFinger_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, 480, 150);
  pwm.setPWM(4, 0, position);
}

void pinkyCallback(const std_msgs::Float64 &pinkyFinger_msg)
{
  int angle = map((abs(pinkyFinger_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, 480, 150);
  pwm.setPWM(3, 0, position);
}

void armCallback(const std_msgs::Float64 &arm_msg)
{
  int angle = map((abs(arm_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(5, 0, position);
  pwm.setPWM(6, 0, position);
}

void wristCallback(const std_msgs::Float64 &wrist_msg)
{
  int angle = map((abs(wrist_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(5, 0, position);
  pwm.setPWM(6, 0, position);
}

/*ROS SUBSCRIBERS*/
ros::Subscriber<std_msgs::Float32> frontWheelLeft("motor/front/left", frontLeftCb);
ros::Subscriber<std_msgs::Float32> frontWheelRight("motor/front/right", frontRightCb);
ros::Subscriber<std_msgs::Float32> rearWheelLeft("motor/rear/left", rearLeftCb);
ros::Subscriber<std_msgs::Float32> rearWheelRight("motor/rear/right", rearRightCb);
ros::Subscriber<std_msgs::Float64> thumbServo("/Tayson/BASE_FOREARM_position_controller/command", thumbCallback);
ros::Subscriber<std_msgs::Float64> indexServo("/Tayson/BASE_FOREARM_position_controller/command", indexCallback);
ros::Subscriber<std_msgs::Float64> middleServo("/Tayson/BASE_FOREARM_position_controller/command", middleCallback);
ros::Subscriber<std_msgs::Float64> ringServo("/Tayson/BASE_FOREARM_position_controller/command", ringCallback);
ros::Subscriber<std_msgs::Float64> pinkyServo("/Tayson/BASE_FOREARM_position_controller/command", pinkyCallback);
ros::Subscriber<std_msgs::Float64> armServo("/Tayson/BASE_FOREARM_position_controller/command", armCallback);
ros::Subscriber<std_msgs::Float64> wristServo("/Tayson/BASE_FOREARM_position_controller/command", wristCallback);

void setup()
{
  pinMode(d1EN_A, OUTPUT);
  pinMode(d1IN1_A, OUTPUT);
  pinMode(d1IN2_A, OUTPUT);
  pinMode(d1IN1_B, OUTPUT);
  pinMode(d1IN2_B, OUTPUT);
  pinMode(d1EN_B, OUTPUT);
  pinMode(d2EN_A, OUTPUT);
  pinMode(d2IN1_A, OUTPUT);
  pinMode(d2IN2_A, OUTPUT);
  pinMode(d2IN1_B, OUTPUT);
  pinMode(d2IN2_B, OUTPUT);
  pinMode(d2EN_B, OUTPUT);

  // SERVO DRIVER
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ); // Analog servos run at ~50 Hz updates
  pwm.setPWM(0, 0, 480);
  pwm.setPWM(1, 0, 100);
  pwm.setPWM(2, 0, 480);
  pwm.setPWM(3, 0, 480);
  pwm.setPWM(4, 0, 480);
  pwm.setPWM(5, 0, (2100 - 750) / 2);
  pwm.setPWM(6, 0, (2100 - 750) / 2);
  pwm.setPWM(7, 0, (2100 - 600) / 2);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(frontWheelLeft);
  nh.subscribe(frontWheelRight);
  nh.subscribe(rearWheelLeft);
  nh.subscribe(rearWheelRight);
  nh.subscribe(thumbServo);
  nh.subscribe(indexServo);
  nh.subscribe(middleServo);
  nh.subscribe(ringServo);
  nh.subscribe(pinkyServo);
  nh.subscribe(armServo);
  nh.subscribe(wristServo);

  nh.logdebug("Started");
  BTSerial.begin(115200);
}

void loop()
{
  nh.spinOnce();
}