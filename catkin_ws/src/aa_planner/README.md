# Assured Autonomy - Robot Planner

This module integrates trained planners from [aa\_simulation](https://github.com/r-pad/aa_simulation) into the software architecture of the [FFAST](https://github.com/jsford/FFAST) RC vehicle. Specifically, this module allows the planner trained from simulation to be run on the FFAST vehicle using ROS Kinetic. ROS is the only dependency.

## Code Structure

The ```nodes``` directory contains the file ```nodes/planner``` which acts as a ROS wrapper for the planner. In other words, it subscribes to localization messages and publishes action commands from the trained policy. This is the only file that calls ROS functions.

The ```src``` directory contains the file ```src/aa_planner/policy.py``` which reads from the trained policies. Files in this directory are not dependent on ROS.

## Instructions

Save trained models into the directory ```src/aa_planner/```. The circle planner must be saved with the name ```circle_model.save``` and the straight line planner must be saved with the name ```straight_model.save```.

## Usage

There are three possible modes available: ```straight```, ```circle``` and ```rounded_square```. The first two modes command the car to move in a straight line and a circle, respectively. The last mode integrates the two trained planners to command the car to move in a trajectory that resembles a square with rounded, circular corners. Use the following command to run the planner on the robot:

```
roslaunch aa_planner planner.launch mode:={straight, circle, rounded_square}
```
