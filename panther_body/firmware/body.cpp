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
//SFR10 sensors[SIZE_SFR10] = {SFR10(112, "sfr10_left"), SFR10(113, "sfr10_right")};

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
ros::Publisher pub_range_left("sfr10_left", &range_msg);
ros::Publisher pub_range_right("sfr10_right", &range_msg);
// Define the twist subscriber
ros::Subscriber<geometry_msgs::Twist> twist(TWIST_SUBSCRIBER_TOPIC, &TwistMessageCb);
ros::Subscriber<std_msgs::Bool> enable_status(ENABLE_SUBSCRIBER_TOPIC, &EnableMessageCb);

char frame_left[] = "frame_left";
char frame_right[] = "frame_right";
SFR10_t sensors[SIZE_SFR10];

void setup()
{
  // Initializtion ROS node
  nh.initNode();
  // Initialization LED builtin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialization NeoPixels
  pinMode(LEDS_LEFT, OUTPUT);
  pinMode(LEDS_RIGHT, OUTPUT);
  // Initialize SFR10 sensors
  SFR10_connect(&range_msg);
  SFR10_init(&sensors[0], &pub_range_left, &range_msg, 112, frame_left);
  SFR10_init(&sensors[1], &pub_range_right, &range_msg, 113, frame_right);

  // Publish Sonar range
  nh.advertise(pub_range_left);
  nh.advertise(pub_range_right);

  // Initialization twist and enable subscriber
  nh.subscribe(twist);
  nh.subscribe(enable_status);
}

bool status = true;
unsigned long range_timer;



void loop()
{
  // publish the range value every 50 milliseconds
  //   since it takes that long for the sensor to stabilize
  if ( (millis() - range_timer) > 50){
    // Update led status
    digitalWrite(LED_BUILTIN, status);
    //range_msg.range = sensors[0].getRange();
    int left = SFR10_publish(&nh, &sensors[0]);
    //int right = SFR10_publish(&nh, &sensors[1]);
    // Update range timer
    range_timer =  millis();
    status = !status;
  }
  nh.spinOnce();
}
// EOF

