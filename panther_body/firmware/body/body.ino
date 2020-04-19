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

//#define LED_BUILTIN           13 //ALREADY DEFINED
#define STRIP_LEFT              3
#define STRIP_RIGHT             4
// NeoPixels attached
#define STRIP_NUM               43
#define STRIP_DEF_BRIGHTNESS    50
// Effects
#define STRIP_SONAR             0
#define STRIP_RAINBOW           1
// Topic definition
#define SUBSCRIBER_TWIST "cmd_vel"
#define SUBSCRIBER_STOP  "e_stop"
#define SUBSCRIBER_ENABLE "enable"

// ROS libraries
#include <ros.h>
// http://wiki.ros.org/std_msgs
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
// https://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
#include <geometry_msgs/Twist.h>
// http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
#include <sensor_msgs/Range.h>
// I2C library
#include <Wire.h>
// Load Arduino_NeoPixel ligray
// https://github.com/adafruit/Adafruit_NeoPixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

char loginfo_buffer[100];
// Functions definitions
void TwistMessageCb( const geometry_msgs::Twist& msg);
void EnableMessageCb(const std_msgs::Int8& msg);
void StopMessageCb(const std_msgs::Bool& msg);


// Soft timers
typedef struct _soft_timer {
  bool start;
  unsigned long currentMillis;
  unsigned long previousMillis;
  unsigned long interval;
} soft_timer_t;
// Publish timer
soft_timer_t publish, sfr10_update;
// Initialize ROS
ros::NodeHandle  nh;
// Neopixel
typedef struct _neo_pixel {
  // Parameter 1 = number of pixels in strip
  // Parameter 2 = Arduino pin number (most are valid)
  // Parameter 3 = pixel type flags, add together as needed:
  //   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
  //   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
  //   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
  //   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
  //   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
  Adafruit_NeoPixel NEOPixel;  // NEOPIXEL line definition
  unsigned int brightness;
  struct {
    bool enable;
    unsigned int start;
    unsigned int end;
    uint32_t color;
  } range;
  struct {
    bool enable;
    uint32_t color;
    bool status;
    unsigned int step;
    int counter;
    int newbrightness;
  } bright;
  struct {
    bool enable;
    unsigned int counter;
  } rainbow;
} neo_pixel_t;
neo_pixel_t strip_left = {Adafruit_NeoPixel(STRIP_NUM, STRIP_LEFT, NEO_GRB + NEO_KHZ800),
                          STRIP_DEF_BRIGHTNESS};
neo_pixel_t strip_right = {Adafruit_NeoPixel(STRIP_NUM, STRIP_RIGHT, NEO_GRB + NEO_KHZ800),
                           STRIP_DEF_BRIGHTNESS};
// SFR10 definition
typedef struct _SFR10 {
    int address;
    char frame_id[15];
    ros::Publisher* pub;
    sensor_msgs::Range* msg;
    int distance;
} SFR10_t;
// Message
sensor_msgs::Range range_msg;
// Initialization SFR10 sensors
#define SFR10_SIZE      3
#define SFR10_LEFT      0
#define SFR10_RIGHT     1
#define SFR10_REAR      2
ros::Publisher pub_range_left("sfr10_left", &range_msg);
ros::Publisher pub_range_right("sfr10_right", &range_msg);
ros::Publisher pub_range_rear("sfr10_rear", &range_msg);
SFR10_t sensors[SFR10_SIZE] = {
  {112, "frame_left", &pub_range_left, &range_msg, 0},
  {113, "frame_right", &pub_range_right, &range_msg, 0},
  {114, "frame_rear", &pub_range_rear, &range_msg, 0}};
// Subscribers update
ros::Subscriber<geometry_msgs::Twist> sub_twist(SUBSCRIBER_TWIST, &TwistMessageCb);
ros::Subscriber<std_msgs::Bool> sub_stop(SUBSCRIBER_STOP, &StopMessageCb);
ros::Subscriber<std_msgs::Int8> sub_enable(SUBSCRIBER_ENABLE, &EnableMessageCb);

geometry_msgs::Twist twist;
int enable_status = 3;
bool stop_status = false;

/**
   @brief TwistMessageCb Twist message callback and conversion linear and angular velocity in led effect
*/
void TwistMessageCb( const geometry_msgs::Twist& msg) {
  // Save linear velocity
  twist = msg;
  // LOG linear and angular
  sprintf(loginfo_buffer, "Twist: lin:%.2f ang:%.2f", twist.linear.x, twist.angular.z);
  nh.loginfo(loginfo_buffer);
}
/**
 * @brief Enable message
 */

void EnableMessageCb(const std_msgs::Int8& msg) {
  // Update status controller
  enable_status = msg.data;
  // Send log message information
  sprintf(loginfo_buffer, "Enable: %d", enable_status);
  nh.loginfo(loginfo_buffer);
}
/**
 * @brief Stop message
 */
void StopMessageCb(const std_msgs::Bool& msg) {
  // Update status controller
  stop_status = msg.data;
  // Send log message information
  sprintf(loginfo_buffer, "Stop: %d", stop_status);
  nh.loginfo(loginfo_buffer);
}


void setup()
{
  // Initialization LED builtin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialization NeoPixels
  NEOpixel_init(strip_left);
  NEOpixel_init(strip_right);
  // Initialize ROS and topics
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.subscribe(sub_enable);
  nh.subscribe(sub_stop);
  // Connect SFR10
  SFR10_connect(&range_msg);
  // Initialize all sensors
  for(int i = 0; i < SFR10_SIZE; ++i)
  {
    SFR10_init(&nh, sensors[i]);
  }
  // Initialize soft timer
  soft_timer_init(sfr10_update, 0.125);
  soft_timer_init(publish, 0.500);
}


bool led_status = true;


void loop()
{
  // Clear status strips
  NEOpixel_clear(strip_left);
  NEOpixel_clear(strip_right);
  switch(enable_status)
  {
    case STRIP_RAINBOW:
      NEOpixel_rainbow(strip_left);
      NEOpixel_rainbow(strip_right);
      break;
    default:
      NEOpixel_rainbow_stop(strip_left);
      NEOpixel_rainbow_stop(strip_right);
  }
  // Fast loop update SFR10 sensor
  if (soft_timer_run(sfr10_update))
  {
    // Update SFR10 status
    for(int i = 0; i < SFR10_SIZE; ++i)
    {
      SFR10_update(sensors[i]);
    }
    // Run only if is not in stop
    if(enable_status == STRIP_SONAR)
    {
      NEOpixel_range(strip_left, strip_left.NEOPixel.Color(255, 0, 0), sensors[SFR10_LEFT].distance / 2, true);
      NEOpixel_range(strip_right, strip_right.NEOPixel.Color(255, 0, 0), sensors[SFR10_RIGHT].distance / 2, true);
      NEOpixel_range(strip_left, strip_left.NEOPixel.Color(255, 0, 0), sensors[SFR10_REAR].distance / 2, false);
      NEOpixel_range(strip_right, strip_right.NEOPixel.Color(255, 0, 0), sensors[SFR10_REAR].distance / 2, false);
    } else {
      NEOpixel_range_stop(strip_left);
      NEOpixel_range_stop(strip_right);
    }
  }
  // Slow loop publish SFR10 status
  if (soft_timer_run(publish))
  {
    // Update led status
    digitalWrite(LED_BUILTIN, led_status);
    led_status = !led_status;
    // Update SFR10 status
    for(int i = 0; i < SFR10_SIZE; ++i)
    {
      SFR10_publish(&nh, sensors[i]);
    }
  }
  // Run only if is not in stop
  if(stop_status)
  {
    NEOpixel_bright(strip_left, strip_left.NEOPixel.Color(255, 0, 0), 250);
    NEOpixel_bright(strip_right, strip_right.NEOPixel.Color(255, 0, 0), 250);
  } else {
    NEOpixel_bright_stop(strip_left);
    NEOpixel_bright_stop(strip_right);
  }
  // Update neopixels
  NEOpixel_update(strip_left);
  NEOpixel_update(strip_right);
  // ROS spin update
  nh.spinOnce();
}
