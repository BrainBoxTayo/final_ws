#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import bluetooth

#Bluetooth MAC address of the ESP32
ESP32_MAC_ADRESS = "D4:8A:FC:CF:AC:C6"
port = 1

# connect to the ESP32 using rfcomm
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((ESP32_MAC_ADRESS, port))

latest_velocities = {}

def callback(data, name):
    # Update the latest velocity for the motor
    
    latest_velocities[name] = data.data
    velocities_str = ""
    message = ""   
    # Store the messages in  a string
    for name, velocity in latest_velocities.items():
        velocities_str += "{}, {}".format(name, velocity)
        message += "{} ".format(velocity)
    # Print the latest velocities without changing lines
    print("\r{}".format(velocities_str), end="")
    # send to the ESP
    sock.send(message + '\n')

def listener():
    rospy.init_node('motor_listener', anonymous=True)

    rospy.Subscriber('motor/front/left', Float32, lambda data: callback(data, "motor/front/left"))
    rospy.Subscriber('motor/front/right', Float32, lambda data: callback(data, "motor/front/right"))
    rospy.Subscriber('motor/rear/left', Float32, lambda data: callback(data, "motor/rear/left"))
    rospy.Subscriber('motor/rear/right', Float32, lambda data: callback(data, "motor/rear/right"))

    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    finally:
        #close bluetooth socket when done
        sock.close()
