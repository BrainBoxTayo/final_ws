#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

latest_velocities = {}

def callback(data, name):
    # Update the latest velocity for the motor
    latest_velocities[name] = data.data
    
    # Print the latest velocities without changing lines
    velocities_str = ", ".join("{}: {:.2f}".format(name, vel) for name, vel in latest_velocities.items())
    print("\rLatest motor velocities: {}".format(velocities_str), end="")


def listener():
    rospy.init_node('motor_listener', anonymous=True)

    rospy.Subscriber('motor/front/left', Float32, lambda data: callback(data, "motor/front/left"))
    rospy.Subscriber('motor/front/right', Float32, lambda data: callback(data, "motor/front/right"))
    rospy.Subscriber('motor/rear/left', Float32, lambda data: callback(data, "motor/rear/left"))
    rospy.Subscriber('motor/rear/right', Float32, lambda data: callback(data, "motor/rear/right"))

    rospy.spin()

if __name__ == '__main__':
    listener()

