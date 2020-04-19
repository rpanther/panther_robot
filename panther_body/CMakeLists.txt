cmake_minimum_required(VERSION 2.8.3)
project(panther_body)

# REFERENCE
# http://wiki.ros.org/rosserial_arduino/Tutorials/CMake

find_package(catkin REQUIRED COMPONENTS 
  ros_teensy
  rosserial_arduino 
  rosserial_client
  
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs
    std_msgs
    sensor_msgs
)

###########
## Build ##
###########

rosserial_generate_ros_lib(
  PACKAGE rosserial_arduino
  SCRIPT make_libraries.py
)

rosserial_configure_client(
  DIRECTORY firmware
  TOOLCHAIN_FILE ${ROS_TEENSY_TOOLCHAIN}
)

# TODO: Removed waiting NUCLEO to change
# rosserial_add_client_target(firmware body_Firmware ALL)
# rosserial_add_client_target(firmware body-upload)

