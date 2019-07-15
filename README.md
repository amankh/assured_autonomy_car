# FFAST

## Introduction

The FFAST vehicle is a 1:10 scale RC car functioning as a test platform for testing dynamic motion planning and control algorithms. This repository contains the software developed throughout the DARPA Assured Autonomy project at Carnegie Mellon University, including the source codes in the vehicle responsible for autonomous driving, executing maneuvers, and verification.

## Functionalities

The rear-wheel drive front steering vehicle is capable of driving with speed of up to 6 m/s forward and 3 m/s backwards and achieving steering angles of 30 degrees on either side. It achieves localization reasonably well using data from the Hall effect sensors on the motor as odometry, IMU, LIDAR and the camera on the Jetson developer kit with optic flow. It can be teleoperated or controlled by a software giving it navigation commands. A

## Organization

The repository is divided into several subfolders:
- doc  
Directory for instructions for developing the RC car platform and the detailed documentation of the project
- catkin_ws  
Catkin workspace containing the ROS packages that control the vehicle
- matlab_simulator  
MATLAB scripts for simulation of vehicle dynamics and execution of the maneuvers in simulation

## Getting Started

The [setup instructions](doc/setup_instructions.md) will get you a copy of the project up and running on your local machine for development and testing purposes. It has been developed for Linux, and no cross-compatibility effort has been made.

## Running the tests
Refer to the [test runs](doc/test_runs.md) for the commands to run the tests.





## Acknowledgments

* Carnegie Mellon University
* DARPA Assured Autonomy Program 

