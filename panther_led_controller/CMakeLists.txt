cmake_minimum_required(VERSION 2.8.3)
project(panther_led_controller)

# REFERENCE
# http://wiki.ros.org/rosserial_arduino/Tutorials/CMake

find_package(catkin REQUIRED COMPONENTS 
#    rosserial_arduino 
    rosserial_client
)

catkin_package()

###########
## Build ##
###########

#rosserial_generate_ros_lib(
#  PACKAGE rosserial_arduino
#  SCRIPT make_libraries.py
#)

#rosserial_configure_client(
#  DIRECTORY firmware
#  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
#)

#rosserial_add_client_target(firmware hello ALL)
#rosserial_add_client_target(firmware hello-upload)

#################################
## ARDUINO - Compile and build ##
#################################

#include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
#add_definitions(-DUSB_CON)

#generate_arduino_firmware(hello
#  SRCS chatter.cpp ${ROS_LIB_DIR}/time.cpp
#  BOARD leonardo
#  PORT /dev/ttyACM0
#)
