/**
   Copyright (C) 2017, Raffaello Bonghi <raffaello@rnext.it>
   All rights reserved

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
   BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
   HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
   OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
   WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
   OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
   EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// -------------------------//
//       LED DEFINITION     //
// -------------------------//

// Led definition
#define PIN_LEFT            2
#define PIN_RIGHT           3
#define LED_GREEN          10
// NeoPixels attached
#define NUMPIXELS          43

#define DEFAULT_LINE_LNG   NUMPIXELS
#define DEFAULT_LINE_VEL   0.5          //[m/s]
#define DEFAULT_LINE_ANG   0          //[rad/s]

// Topic definition
#define TWIST_SUBSCRIBER_TOPIC "/cmd_vel"
// -------------------------//

// Teensy Reset REFERENCE https://forum.pjrc.com/threads/44857-How-to-Reset-Restart-Teensy-3-5-using-sotware
#define RESTART_ADDR 0xE000ED0C
#define READ_RESTART() (*(volatile uint32_t *)RESTART_ADDR)
#define WRITE_RESTART(val) ((*(volatile uint32_t *)RESTART_ADDR) = (val))

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include "TimerOne.h"
#include "TimerThree.h"
// ROS header
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/ColorRGBA.h>

#define L1_POS             0         //10
#define L2_POS             NUMPIXELS //NUMPIXELS-10
#define NEOPIXEL_LNG_MM    5.0                                     //Length of a single pixel
// pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
#define NEOPIXEL_LED_OFF   Adafruit_NeoPixel::Color(0, 0, 0)       // Default LED OFF
#define NEOPIXEL_LED_ON    Adafruit_NeoPixel::Color(255, 255, 255) // Default WHITE ON

typedef struct _soft_timer {
  bool start;
  unsigned long currentMillis;
  unsigned long previousMillis;
  long interval;
} soft_timer_t;

typedef struct _neo_pixel_swipe {
  // When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
  // Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
  Adafruit_NeoPixel *NEOPixel;  // NEOPIXEL line definition
  int length;
  int start;
  int stop;
  uint32_t bg_color;
  int idx;
} neo_pixel_swipe_t;

// ------ VARIABLES ---------//

// ROS Node Handle
ros::NodeHandle  nh;

// NEO Pixel definition
Adafruit_NeoPixel neo_pixel_left  = Adafruit_NeoPixel(NUMPIXELS, PIN_LEFT,  NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel neo_pixel_right = Adafruit_NeoPixel(NUMPIXELS, PIN_RIGHT, NEO_GRB + NEO_KHZ800);

neo_pixel_swipe_t line_swipe_left; // Left line definition
neo_pixel_swipe_t line_swipe_right; // Right line definition
int color_idx_left = 0, color_idx_right = 0;

// Velocity of line swipe
volatile float line_vel = DEFAULT_LINE_VEL;
volatile float line_ang = DEFAULT_LINE_ANG;
float k_v = 1;
float k_w = 1;

// LED colors sequence
#define INTENSITY          50
uint32_t colors[5];
// Size of LED colors
int size_colors = 0;

soft_timer_t timer;
soft_timer_t twist_check;

// ------SUBSCRIBER --------//

char tmp_buffer[50];
/**
   @brief TwistMessageCb Twist message callback and conversion linear and angular velocity in led effect
*/
void TwistMessageCb( const geometry_msgs::Twist& msg) {
  // Save linear velocity
  line_vel = msg.linear.x;
  line_ang = msg.angular.z;
  // Reset timer check twist message
  soft_timer_start(twist_check);
  // LOG linear and angular
  sprintf(tmp_buffer, "lin:%.2f ang:%.2f", msg.linear.x, msg.angular.z);
  nh.logdebug(tmp_buffer);
}

// Define the twist subscriber
ros::Subscriber<geometry_msgs::Twist> sub(TWIST_SUBSCRIBER_TOPIC, &TwistMessageCb );

// --------SETUP ----------//

void setup() {
  // Initialization LED builtin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialization Green strip LED
  pinMode(LED_GREEN, OUTPUT);
  // Set low LED
  digitalWrite(LED_GREEN, LOW);

  // Initialization ROS node
  nh.initNode();
  // Initialization subscriber
  nh.subscribe(sub);
  
  // Wait connection from ROS
  while (!nh.connected()) {
    nh.spinOnce();
  }
  nh.loginfo("Connected!");

  // Reference Parameters http://wiki.ros.org/rosserial/Overview/Parameters
  // Get param gain linear velocity
  if (!nh.getParam("~k_v", &k_v, 1)) {
    nh.logwarn("Load default k_v = 1");
    k_v = 1;
  }
  // Get param gain angular velocity
  if (!nh.getParam("~k_w", &k_w, 1)) {
    nh.logwarn("Load default k_w = 1");
    k_w = 1;
  }

  nh.loginfo("Parameters loaded");

  // Initialization swipe line
  led_swipe_init(line_swipe_left, &neo_pixel_left, L1_POS, L2_POS, DEFAULT_LINE_LNG, NEOPIXEL_LED_OFF);
  led_swipe_init(line_swipe_right, &neo_pixel_right, L1_POS, L2_POS, DEFAULT_LINE_LNG, NEOPIXEL_LED_OFF);

  Timer1.initialize(500000);             // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callbackLeft);  // attaches callback() as a timer overflow interrupt

  Timer3.initialize(500000);              // initialize timer1, and set a 1/2 second period
  Timer3.attachInterrupt(callbackRight);  // attaches callback() as a timer overflow interrupt

  Timer1.stop();
  Timer3.stop();

  soft_timer_init(timer, 1.0);
  // twist check controller
  soft_timer_init(twist_check, 5.0);
  soft_timer_stop(twist_check);

  // Load default led configuration
  colors[0] = Adafruit_NeoPixel::Color(0, INTENSITY, 0);
  colors[1] = Adafruit_NeoPixel::Color(INTENSITY, 0, 0);
  colors[2] = Adafruit_NeoPixel::Color(0, 0, INTENSITY);
  colors[3] = Adafruit_NeoPixel::Color(INTENSITY, INTENSITY, 0);
  colors[4] = Adafruit_NeoPixel::Color(INTENSITY, 0, INTENSITY);
  // Evaluate size of colors led array
  size_colors = sizeof(colors) / sizeof(uint32_t);
}

// --------LOOP ----------//

volatile float vel_left = line_vel;
volatile float vel_right = line_vel;
volatile float vel_left_old = vel_left;
volatile float vel_right_old = vel_right;

void loop() {

  // Check if ROS is connected
  if (!nh.connected()) {
    // Clear line
    led_swipe_reset(line_swipe_left);
    led_swipe_reset(line_swipe_right);
    // reset Board
    WRITE_RESTART(0x5FA0004);
  }

  if (soft_timer_run(timer)) {
    // set the LED with the ledState of the variable:
    digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
  }

  // If any message as received in N time. The swipe reset and wait a new message
  if (soft_timer_run(twist_check)) {
    // Send log message
    nh.loginfo("No twist message for 5 seconds");
    // reset swipe effect
    led_swipe_reset(line_swipe_left);
    led_swipe_reset(line_swipe_right);
    // stop twist check timer
    soft_timer_stop(twist_check);
  }

  vel_left = k_v * line_vel + k_w * line_ang;
  vel_right = k_v * line_vel - k_w * line_ang;

  // Update period timer
  if (vel_left != vel_left_old) {
    if (vel_left != 0) {
      Timer1.resume();
      Timer1.setPeriod((NEOPIXEL_LNG_MM * 1000.0) / fabs(vel_left));
    } else {
      Timer1.stop();
    }
    vel_left_old = vel_left;
  }
  if (vel_right != vel_right_old) {
    if (vel_right != 0) {
      Timer3.resume();
      Timer3.setPeriod((NEOPIXEL_LNG_MM * 1000.0) / fabs(vel_right));
    } else {
      Timer3.stop();
    }
    vel_right_old = vel_right;
  }
  // Spin once ROS
  nh.spinOnce();
}

// -------------------------//

/**
   @brief Timer callback to update velocity for neo pixel line left
*/
void callbackLeft() {
  if (vel_left != 0) {
    // Launch swipe effect with the color on the list
    color_idx_left += led_swipe_run(line_swipe_left, colors[color_idx_left], vel_left);
    // Update color index
    if (color_idx_left >= size_colors) {
      color_idx_left = 0;
    } else if (color_idx_left < 0) {
      color_idx_left = size_colors - 1;
    }
  }
}
/**
   @brief Timer callback to update velocity for neo pixel line right
*/
void callbackRight() {
  if (vel_right != 0) {
    // Launch swipe effect with the color on the list
    color_idx_right += led_swipe_run(line_swipe_right, colors[color_idx_right], vel_right);
    // Update color index
    if (color_idx_right >= size_colors) {
      color_idx_right = 0;
    } else if (color_idx_right < 0) {
      color_idx_right = size_colors - 1;
    }
  }
}

