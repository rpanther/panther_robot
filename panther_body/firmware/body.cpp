/**
 * Copyright (C) 2020, Raffaello Bonghi <raffaello@rnext.it>
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its 
 *    contributors may be used to endorse or promote products derived 
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

// Use the following line if you have a Leonardo or MKR1000
// #define USE_USBCON

// ROS
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>
// Neopixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
// I2C - Sonar SFR10
#include <Wire.h>
// Local imports
#include "SFR10.hpp"

//ALREADY DEFINED
//#define LED_BUILTIN      13
#define LEDS_LEFT          3
#define LEDS_RIGHT         4
// NeoPixels attached
#define NUMPIXELS          43
// Number Ultrasounds SFR10
#define SIZE_SFR10         2

// Topic definition
#define TWIST_SUBSCRIBER_TOPIC "/cmd_vel"
#define ENABLE_SUBSCRIBER_TOPIC "~status"

// Initialization ROS
ros::NodeHandle nh;
// Initialization sensors
//SFR10 sensors[SIZE_SFR10] = {SFR10(112, "sfr10_left"), SFR10(113, "sfr10_right"), SFR10(113, "sfr10_rear")};
SFR10 sensors[SIZE_SFR10] = {SFR10(112, "sfr10_left"), SFR10(113, "sfr10_right")};

void TwistMessageCb( const geometry_msgs::Twist& msg) {
  char tmp_buffer[50];
  // LOG linear and angular
  sprintf(tmp_buffer, "lin:%.2f ang:%.2f", msg.linear.x, msg.angular.z);
  // nh.logdebug(tmp_buffer);
  nh.loginfo(tmp_buffer);
}

void EnableMessageCb( const std_msgs::Bool& msg) {
  // Update status controller
  //controller_run = msg.data;
  // Send log message information
  //nh.logdebug("Bool status");
  nh.loginfo("Bool status");
}

// Reference Range message
// http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
sensor_msgs::Range range_msg;
// Define the twist subscriber
ros::Subscriber<geometry_msgs::Twist> twist(TWIST_SUBSCRIBER_TOPIC, &TwistMessageCb);
ros::Subscriber<std_msgs::Bool> enable_status(ENABLE_SUBSCRIBER_TOPIC, &EnableMessageCb);
ros::Publisher pub_range("range_data", &range_msg);


void setup()
{
  // Initialization LED builtin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialization NeoPixels
  pinMode(LEDS_LEFT, OUTPUT);
  pinMode(LEDS_RIGHT, OUTPUT);
  // Waits to make sure everything is powered up before sending or receiving data
  Wire.begin();
  // Initializtion ROS node
  nh.initNode();
  // Initialization twist and enable subscriber
  nh.subscribe(twist);
  nh.subscribe(enable_status);
  // Publish Sonar range
  nh.advertise(pub_range);

  
  // Initialzie range message
  // SFR10 https://www.robot-electronics.co.uk/htm/srf10tech.htm
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  // range_msg.header.frame_id =  frameid;
  range_msg.field_of_view = 0.01;
  range_msg.min_range = 0.03;
  range_msg.max_range = 0.4;
}

bool status = true;

void loop()
{
  digitalWrite(LED_BUILTIN, status);
  for(int i = 0; i < SIZE_SFR10; ++i)
  {
    sensors[i].getRange();
  }
  status = !status;
  delay(500);
}
// EOF

