cmake_minimum_required(VERSION 2.8.3)

# REFERENCE
# http://wiki.ros.org/rosserial_arduino/Tutorials/CMake
# https://github.com/queezythegreat/arduino-cmake#creating-firmware-images
#set(led_controller_SRCS chatter.cpp ${ROS_LIB_DIR}/time.cpp)
set(led_controller_SRCS led_controller/led_controller.ino led_controller/led_effect.ino led_controller/system.ino ${ROS_LIB_DIR}/time.cpp)
#set(led_controller_HDRS  chatter.h)
set(led_controller_PORT /dev/ttyUSB0)
# Arduino nano328 is the same of uno!
# See: http://code.google.com/p/arduino/wiki/Platforms
set(led_controller_BOARD uno)

include_directories(${ROS_LIB_DIR})
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

# Set author to download
set(GIT_AUTHOR "adafruit")
set(GIT_REPO "Adafruit_NeoPixel")
# Check existence library
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/libraries/${GIT_REPO}")
if (NOT EXISTS "${PATH}")
  message(STATUS "Library does not exist. Downloading now ...")
  # https://stackoverflow.com/questions/35689501/cmakes-execute-process-and-arbitrary-shell-scripts
  execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/arduino_libraries.sh ${CMAKE_CURRENT_SOURCE_DIR} ${GIT_AUTHOR} ${GIT_REPO})
endif()

# Link directories libraries
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
# Generate arduino firmware
generate_arduino_firmware(led_controller)
