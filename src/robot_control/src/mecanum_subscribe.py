#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32, Float64
import bluetooth

#Bluetooth MAC address of the ESP32
# ESP32_MAC_ADRESS = "D4:8A:FC:CF:AC:C6"
ESP32_MAC_ADRESS = "D4:8A:FC:A8:96:7A"
port = 1

# connect to the ESP32 using rfcomm
sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
sock.connect((ESP32_MAC_ADRESS, port))

latest_velocities = {}
latest_positions = {}
prev_data = 0.0 #used to check if there are updates in hand movements

def callback(data, name):
    
    # Update the latest velocity for the motor
    
    latest_velocities[name] = data.data
    velocities_str = ""
    message=""
    # Store the messages in  a string
    for name, velocity in latest_velocities.items():
        velocities_str += "{}, {} ".format(name, velocity)
    # Debugger: Print the latest velocities without changing lines
    print("\r{} ".format(velocities_str), end="")
    # send to ESP
    sock.send("{} \n".format(velocities_str))


def servoCallback(data, name):
    """
    Callback function to handle incoming servo position data.

    Parameters:
    data (object): The new position data for the servo.
    name (str): The identifier for the servo.
    """
    global prev_data
    latest_positions[name] = data.data
    positions_str = ""
    message = ""
    for name, position in latest_positions.items():
        positions_str += "{}, {} ".format(name, position)
    # Print the latest positions without changing lines for debugging
    print("\r{} ".format(positions_str), end="")
    # send to the ESP formatted e.g 1 1.57\n
    sock.send("{} \n".format(positions_str))

def listener():
    rospy.init_node('motor_listener', anonymous=False)

    rospy.Subscriber('motor/front/left', Float32, lambda data: callback(data, "fl"))
    rospy.Subscriber('motor/front/right', Float32, lambda data: callback(data, "fr"))
    rospy.Subscriber('motor/rear/left', Float32, lambda data: callback(data, "rl"))
    rospy.Subscriber('motor/rear/right', Float32, lambda data: callback(data, "rr"))
    rospy.Subscriber('/Tayson/BASE_FOREARM_position_controller/command', Float64, lambda data: servoCallback(data, "1"))
    rospy.Subscriber('/Tayson/hSERV_HAND_position_controller/command', Float64, lambda data: servoCallback(data, "2"))
    rospy.Subscriber('/Tayson/HAND_pinkybase_position_controller/command', Float64, lambda data: servoCallback(data, "3"))
    rospy.Subscriber('/Tayson/HAND_ringbase_position_controller/command', Float64, lambda data: servoCallback(data, "4"))
    rospy.Subscriber('/Tayson/HAND_middlebase_position_controller/command', Float64, lambda data: servoCallback(data, "5"))
    rospy.Subscriber('/Tayson/HAND_indexbase_position_controller/command', Float64, lambda data: servoCallback(data, "6"))
    # Note the thumb is not included in the simulation due to some URDF issues that I did not have time to resolve
    rospy.Subscriber('/Tayson/HAND_thumbbase_position_controller/command', Float64, lambda data: servoCallback(data, "7"))
    rospy.spin()
    

if __name__ == '__main__':
    try:
        listener()
    finally:
        #close bluetooth socket when done
        sock.close()
