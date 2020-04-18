cmake_minimum_required(VERSION 2.8.3)

# REFERENCE
# http://wiki.ros.org/rosserial_arduino/Tutorials/CMake
# https://github.com/queezythegreat/arduino-cmake#creating-firmware-images
#set(led_controller_SRCS led_controller.cpp system.cpp led_effect.cpp ${ROS_LIB_DIR}/time.cpp)
#set(led_controller_HDRS system.hpp led_effect.hpp)
set(body_SRCS body.cpp SFR10.cpp ${ROS_LIB_DIR}/time.cpp)
set(body_HDRS SFR10.hpp)
set(body_PORT /dev/nano328p)
# Arduino nano328 is the same of uno!
# See: http://code.google.com/p/arduino/wiki/Platforms
set(body_BOARD uno)

include_directories(${ROS_LIB_DIR})
# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

# Set author to download
set(ADAFRUIT_REPO "adafruit/Adafruit_NeoPixel")
set(TIMERONE_REPO "PaulStoffregen/TimerOne")
set(REPOS_LIST ${ADAFRUIT_REPO} ${TIMERONE_REPO})
# Check libraries
message(STATUS "Check Arduino libraries.")
# https://stackoverflow.com/questions/35689501/cmakes-execute-process-and-arbitrary-shell-scripts
execute_process(COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/arduino_libraries.sh ${CMAKE_CURRENT_SOURCE_DIR} ${REPOS_LIST})

# Link directories libraries
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
# Generate arduino firmware
generate_arduino_firmware(body)
