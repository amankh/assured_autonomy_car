# Assured Autonomy - Robot Planner

THIS README IS DEPRECATED. A virtual environment is no longer needed to separately run this planner node. Please see the new README for updated instructions.

This module integrates trained planners into the software architecture of the [FFAST](https://github.com/jsford/FFAST) RC vehicle, designed to run safe motion planning algorithms for the vehicle.

## Disclaimer

Because this vehicle uses an Nvidia Jetson TX2 running Ubuntu 16.04 as its base operating system, it uses ROS Kinetic as a backbone for communications within subsystems of the vehicle. However, ROS Kinetic only supports Python 2 and not Python 3, even though trained models from [aa\_simulation](https://github.com/r-pad/aa_simulation) can only be unpickled using Python 3.

As a result, this ROS node must be run in a separate virtual environment on the robot, so that the node can be run in Python 2. The other ROS nodes should not be run in this virtual environment. To do this, set up a virtual environment using [virtualenv](https://virtualenv.pypa.io/en/latest/), and install necessary but missing Python 3 packages within the environment using pip (and not apt-get) to run this module. You can use the instructions below as reference.

```
pip install virtualenv
virtualenv -p python3 aa_env
pip install pyyaml
pip install rospkg
pip install theano
```

## Usage

Run the command below within the virtual environment set up in the section above. The setup bash script for the catkin workspace should be sourced as well.
```
roslaunch aa_planner planner.launch
```
