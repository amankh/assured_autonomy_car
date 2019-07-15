cmake_minimum_required(VERSION 2.8.3)
project(aa_monitor)

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

# todo: compile bot_arm8.S into bot_arm8.o with gcc -c
#set_property(SOURCE src/monitor/bot_arm8.S PROPERTY LANGUAGE C)
#set_source_files_properties(src/monitor/bot_arm8.S
#    PROPERTIES COMPILE_FLAGS "-x assembler-with-cpp")

#set(ALL_SRC src/utils/utils.cpp 
#            src/monitor/bot_arm8.S)
set(ALL_SRC src/utils/utils.cpp)

add_library(aa_monitor_lib ${ALL_SRC})

add_executable(aa_monitor src/monitor/monitor_node.cpp lib/bot_arm8.o)
target_link_libraries(aa_monitor aa_monitor_lib ${catkin_LIBRARIES})
