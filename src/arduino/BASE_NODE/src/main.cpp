#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
// #include <BluetoothSerial.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <ESP32Servo.h>

#define MAX_SPEED_WHEEL 21
#define MIN_SPEED_WHEEL 0
#define LOW_PWM 100
#define HIGH_PWM 255
#define SERVOMIN 150        // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX 525        // This is the 'maximum' pulse length count (out of 4096)
#define SERVO_FREQ 50       // Analog servos run at ~50 Hz updates
#define SERIAL_SIZE_RX 2048 // should fix the overflow error

// BluetoothSerial BTSerial;
// Initialize Servos
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo armServoRight; // create servo object to control a servo
Servo armServoLeft;
Servo wristServo;

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

/*Servo Pins*/

const int rightPin = 32;
const int leftPin = 33;
const int wristPin = 19;

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

int curr_position_wrist = 90;
int curr_position_arm = 90;

// delay non-blocking
unsigned long current_millis = 0;
unsigned long previous_millis = 0;
unsigned long delay_millis = 500;

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
  pwm.setPWM(3, 0, position);
}

void pinkyCallback(const std_msgs::Float64 &pinkyFinger_msg)
{
  int angle = map((abs(pinkyFinger_msg.data) + 1.57), 1.57, 3.14, 0, 180);
  int position = map(angle, 0, 180, 480, 150);
  pwm.setPWM(4, 0, position);
}

void armCallback(const std_msgs::Float64 &arm_msg)
{
  current_millis = millis();
  if (arm_msg.data >= 0.43) // the arm messages from ROS are inverted man
  {
    if (curr_position_arm > 0 && (current_millis - previous_millis) > delay_millis)
    {
      curr_position_arm--;
      previous_millis = millis();
    }

    nh.loginfo(String(curr_position_arm).c_str());
    armServoLeft.write(curr_position_arm);
    armServoRight.write(180 - curr_position_arm);
  }
  else if (arm_msg.data < -1.43)
  {
    if (curr_position_arm < 180 && (current_millis - previous_millis) > delay_millis)
    {
      curr_position_arm++;
      previous_millis = millis();
    }
    nh.loginfo(String(curr_position_arm).c_str());
    armServoLeft.write(curr_position_arm);
    armServoRight.write((180 - curr_position_arm));
  }
}

void wristCallback(const std_msgs::Float64 &wrist_msg)
{
  current_millis = millis();
  if (wrist_msg.data < -1.43)
  {
    if (curr_position_wrist > 0 && (current_millis - previous_millis) > delay_millis)
    {
      curr_position_wrist--;
      previous_millis = millis();
    }

    nh.loginfo(String(curr_position_wrist).c_str());
    wristServo.write(curr_position_wrist);
  }
  else if (wrist_msg.data > 0.43)
  {
    if (curr_position_wrist < 180 && (current_millis - previous_millis) > delay_millis)
    {
      curr_position_wrist++;
      previous_millis = millis();
    }

    nh.loginfo(String(curr_position_wrist).c_str());
    wristServo.write(curr_position_wrist);
  }
}

/*ROS SUBSCRIBERS*/
ros::Subscriber<std_msgs::Float32> frontWheelLeft("motor/front/left", frontLeftCb);
ros::Subscriber<std_msgs::Float32> frontWheelRight("motor/front/right", frontRightCb);
ros::Subscriber<std_msgs::Float32> rearWheelLeft("motor/rear/left", rearLeftCb);
ros::Subscriber<std_msgs::Float32> rearWheelRight("motor/rear/right", rearRightCb);
ros::Subscriber<std_msgs::Float64> thumbServo("/Tayson/HAND_thumbbase_position_controller/command", thumbCallback);
ros::Subscriber<std_msgs::Float64> indexServo("/Tayson/HAND_indexbase_position_controller/command", indexCallback);
ros::Subscriber<std_msgs::Float64> middleServo("/Tayson/HAND_middlebase_position_controller/command", middleCallback);
ros::Subscriber<std_msgs::Float64> ringServo("/Tayson/HAND_ringbase_position_controller/command", ringCallback);
ros::Subscriber<std_msgs::Float64> pinkyServo("/Tayson/HAND_pinkybase_position_controller/command", pinkyCallback);
ros::Subscriber<std_msgs::Float64> armServo("/Tayson/BASE_FOREARM_position_controller/command", armCallback);
ros::Subscriber<std_msgs::Float64> WristServo("/Tayson/hSERV_HAND_position_controller/command", wristCallback);

void setup()
{
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  wristServo.setPeriodHertz(50);
  armServoLeft.setPeriodHertz(50);
  armServoRight.setPeriodHertz(50);
  wristServo.attach(wristPin, 800, 2800);
  armServoLeft.attach(leftPin, 750, 3000);
  armServoRight.attach(rightPin, 800, 2800);
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
  wristServo.write(curr_position_wrist);
  armServoLeft.write(curr_position_arm);
  armServoRight.write(curr_position_arm);

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
  nh.subscribe(WristServo);

  nh.logdebug("Started");
  Serial.begin(115200);
  Serial.setRxBufferSize(SERIAL_SIZE_RX);
  // BTSerial.begin("BASE_NODE");
}

void loop()
{
  nh.spinOnce();
}