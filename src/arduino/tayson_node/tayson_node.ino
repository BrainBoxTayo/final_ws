/*
 * arduino Node for TAYSON
 */

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Float64.h>
#include <Servo.h>

int pot_pin = A0;
int servo_pin = 9;
double pot_storer = 0.0; 
ros::NodeHandle  nh;
char *nodeName = "/Tayson/BASE_FOREARM_position_controller/command";
Servo servo1;

std_msgs::Float64 pot_msg;
ros::Publisher potentiometer(nodeName, &pot_msg);

void setup()
{
  Serial.begin(57600);
  pinMode(pot_pin, INPUT);
  servo1.attach(servo_pin);
  nh.initNode();
  nh.advertise(potentiometer);  
}

void loop()
{
  pot_storer = map(analogRead(pot_pin), 0, 1023, 0, 180);
  double pot_radians = radians(pot_storer) - HALF_PI;
  pot_msg.data = pot_radians;
  potentiometer.publish( &pot_msg );
  servo1.write(pot_storer); //write  to servo
  nh.spinOnce();
  
} 
