cmake_minimum_required(VERSION 2.8.3)
project(aa_quant_monitor)

add_compile_options(-std=c++11)

# Find build dependencies
find_package(catkin REQUIRED COMPONENTS
    ackermann_msgs
    nav_msgs
    roscpp
    tf2
    diagnostic_msgs
)

# Find runtime dependencies
catkin_package(
    CATKIN_DEPENDS ackermann_msgs nav_msgs roscpp tf2 diagnostic_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
)

set(ALL_SRC src/utils/utils.cpp)
add_library(aa_quant_monitor_lib ${ALL_SRC})

add_executable(aa_quant_monitor src/monitor/monitor_node.cpp)
target_link_libraries(aa_quant_monitor aa_quant_monitor_lib ${catkin_LIBRARIES})
