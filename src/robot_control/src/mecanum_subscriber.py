#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float64
import bluetooth

# Bluetooth MAC address of the ESP32
ESP32_MAC_ADDRESS = "D4:8A:FC:CF:AC:C6"
# ESP32_MAC_ADDRESS = "D4:8A:FC:A8:96:7A"
port = 1

# Connect to the ESP32 using rfcomm
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((ESP32_MAC_ADDRESS, port))

latest_velocities = {}
latest_positions = {}


def send_message(name, data):
    message = "{}, {}\n".format(name, data)
    sock.send(message)
    # Print the message for debugging
    print(message, end='')

def callback(data, name):
    # Update the latest velocity for the motor
    latest_velocities[name] = data.data
    # Send the updated velocity to the ESP
    send_message(name, data.data)

def servoCallback(data, name):
    # Update the latest position for the servo
    latest_positions[name] = data.data
    # Send the updated position to the ESP
    send_message(name, data.data)

def listener():
    rospy.init_node('motor_listener', anonymous=False)

    rospy.Subscriber('motor/front/left', Float32, lambda data: callback(data, "fl"))
    rospy.Subscriber('motor/front/right', Float32, lambda data: callback(data, "fr"))
    rospy.Subscriber('motor/rear/left', Float32, lambda data: callback(data, "rl"))
    rospy.Subscriber('motor/rear/right', Float32, lambda data: callback(data, "rr"))
    rospy.Subscriber('/Tayson/BASE_FOREARM_position_controller/command', Float64, lambda data: servoCallback(data, "6"))
    rospy.Subscriber('/Tayson/hSERV_HAND_position_controller/command', Float64, lambda data: servoCallback(data, "7"))
    rospy.Subscriber('/Tayson/HAND_pinkybase_position_controller/command', Float64, lambda data: servoCallback(data, "1"))
    rospy.Subscriber('/Tayson/HAND_ringbase_position_controller/command', Float64, lambda data: servoCallback(data, "2"))
    rospy.Subscriber('/Tayson/HAND_middlebase_position_controller/command', Float64, lambda data: servoCallback(data, "3"))
    rospy.Subscriber('/Tayson/HAND_indexbase_position_controller/command', Float64, lambda data: servoCallback(data, "4"))
    # Note the thumb is not included in the simulation due to some URDF issues that I did not have time to resolve
    rospy.Subscriber('/Tayson/HAND_thumbbase_position_controller/command', Float64, lambda data: servoCallback(data, "5"))
    
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listener()
    finally:
        # Close Bluetooth socket when done
        sock.close()
