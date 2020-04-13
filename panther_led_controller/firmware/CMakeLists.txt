cmake_minimum_required(VERSION 2.8.3)

include_directories(${ROS_LIB_DIR})

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

# ARDUINO_SDK in /usr/share/arduino
#${ARDUINO_SDK}/libraries
# link_directories(${ARDUINO_SDK}/libraries)

# https://github.com/adafruit/Adafruit_NeoPixel
link_directories(/home/nvidia/Adafruit_NeoPixel)

generate_arduino_firmware(led_controller
  SKETCH led_controller
#  SRCS led_controller/led_controller.ino led_controller/led_effect.ino led_controller/system.ino ${ROS_LIB_DIR}/time.cpp
  BOARD nano
  PORT /dev/nano
)
