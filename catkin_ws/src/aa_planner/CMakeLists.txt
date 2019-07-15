cmake_minimum_required(VERSION 2.8.3)
project(aa_planner)

# Find build dependencies
find_package(catkin REQUIRED COMPONENTS
    ackermann_msgs
    nav_msgs
    rospy
)

# Set up python paths
catkin_python_setup()

# Find runtime dependencies
catkin_package(
    CATKIN_DEPENDS ackermann_msgs nav_msgs rospy
)

# Add other include files
include_directories(
    ${catkin_INCLUDE_DIRS}
)

# Install python scripts
catkin_install_python(PROGRAMS nodes/planner
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
