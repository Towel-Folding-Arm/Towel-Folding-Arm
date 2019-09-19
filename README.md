# Towel-Folding-Arm
A year-long project in which we developed a robot to fold a towel without human interaction.


## Overview

This project consists of three main components: machine vision, robot kinematics, and the control of servo motors.

### Machine Vision

Machine vision is used to process the position and orientation of the towel on a flat surface. To accomplish this, an XBOX 360 Kinect camera mounted on a wooden stand continuously captures color image data and a depth map of the surface. A hough transform is performed on the depth point cloud and used in edge detection to determine the location of the towel's edges and corners. The coordinates of the four corners of the towel are transformed from the camera's reference frame to the robot's reference frame and subsequently published to the motion planning tool.


### Robot Kinematics

The robot arm has 6 degrees of freedom. Four TowerPro MG996R servos control the body of the robot and two smaller TowerPro SG90 servos manipulate the roll and pitch of the end effector. The physical specifications of the robot and rotational limits of each joint are modeled using the Unified Robot Description Format. Coordinates are received from the machine vision publisher and inverse kinematics is computed on the robot model to determine the joint angles that would position the end effector at one of the corners of the towel. These joint angles are converted into pulse widths for each servo and published to the robot.


![kinematics](https://media.githubusercontent.com/media/Towel-Folding-Arm/Towel-Folding-Arm/master/images/kinematics.png)


 
### Servo Control

A Raspberry Pi serves as the controller for the robot arm. The pulse width modulation (PWM) cable of each servo is connected to the general-purpose input/output (GPIO) of the Raspberry Pi and an H bridge controls the opening and closing of the end effector. Each motor has a corresponding Python module on the Raspberry Pi which outputs PWM values to the motor using the pigpio library. The Python multiprocessing library allows the program to change the positions of all servos simultaneously, resulting in fluid motion.

## Demo

![motion](https://media.githubusercontent.com/media/Towel-Folding-Arm/Towel-Folding-Arm/master/images/motion.gif)

## Built with
- [ROS](https://www.ros.org/)
- [OpenCV](https://opencv.org/)
- [MoveIt](https://moveit.ros.org)
- [pigpio](http://abyz.me.uk/rpi/pigpio/python.html)   