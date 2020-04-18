cmake_minimum_required(VERSION 2.8.3)
project(panther_body)

# REFERENCE
# http://wiki.ros.org/rosserial_arduino/Tutorials/CMake

find_package(catkin REQUIRED COMPONENTS 
    rosserial_arduino 
    rosserial_client
    geometry_msgs
    std_msgs
    sensor_msgs
)

catkin_package()

###########
## Build ##
###########

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROSSERIAL_ARDUINO_TOOLCHAIN}
)

rosserial_add_client_target(firmware body ALL)
rosserial_add_client_target(firmware body-upload)

