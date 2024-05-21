# TAYSON
![TAYSON](https://drive.google.com/uc?id=1RBD9NN44qIakStS-U88Z2kEdnXWM7EAg)
About Tayson:
___
About ROS:
ROS refers to the ***Robot Operating System***, which is an environment that connects ***nodes*** together. ***Nodes*** are pieces of software that can be written in C++ or Python. ROS is more than 10 years old and has grown to be the biggest robotics developer community on the planet.
For more on ROS click [here](https://www.ros.org/).
____
## How to get Tayson running on your machine:
- Tayson was made on ROS Noetic on the Ubuntu 20.04 linux distro. To get Ubuntu download this [iso](https://releases.ubuntu.com/focal/ubuntu-20.04.6-desktop-amd64.iso) and follow this [guide](https://www.youtube.com/watch?v=mXyN1aJYefc&t=202s&pp=ygUWdWJ1bnR1IDE4LjA0IGR1YWwgYm9vdA%3D%3D) to dual boot your device. 

- Download ROS ***Noetic Ninjemys*** with this [guide](http://wiki.ros.org/noetic/Installation) from the official website.

- source the ROS setub.bash file using 
```source /opt/ros/noetic/setup.bash``` on the terminal.

- clone this repository to your machine and source the devel setup.bash file 
```source /final_ws/devel/setup.bash```.

## Driving TAYSON in gazebo
- enter the following commands:
 ```
 roslaunch robot_control mecanum.launch
 
 # enter the next line in a new termianl

 rosrun robot_control mecanum_subscribe.py 

 # the line above helps us visualize the wheel velocities.

 # to control the robot using your keyboard, open a new terminal and enter:
 
 rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
___
## Visualizing TAYSON In RVIZ
```
roslaunch robot_description display.launch

# click the add button in rviz and add ROBOT MODEL.
# also change the fixed Frame option to dummy.
```



# NEWMODEL BRANCH

The newmodel branch includes the actual model we are bulding.

on your machine use ```git checkout newmodel```

## viewing TAYSON in rviz
```
roslaunch robot_description display.launch
```
## simulating TAYSON in gazebo
```
roslaunch robot_description spawn_robot_controller.launch

# an rqt_gui window is launched, you can use the message publisher plugin to drive the joints for now
# would add a much better form of control
```
___
# How to use the PS4 controller

- install ds4drv
- launch ds4drv using ```sudo ds4drv```, then press the home and share button on your controller to turn on bluetooth sharing
- when connected it would display a success message
-  ```roslaunch robot_control mecanum.launch```
- press L2 and R2 till the 'calibrated!' message is seen on your terminal
- Drive the robot around using:

    1. **L2**: for forward motion
    1. **R2**: for backward motion
    1. **LS**: Yaxis for Z rotation
    1. **RS**: Zaxis for Strafing in the Y axis
___
## to do list:
- ~~Update the URDF files to the current robot model (we started with a different robot model)~~
- ~~Currently you can also control the robot with a ps4 controller, I would update the README with how to do this later on.~~
- ~~add control for the robot arm.~~
- change effort joint interface to position joint interface