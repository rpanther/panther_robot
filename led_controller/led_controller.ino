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

typedef struct _neo_pixel_swipe {
  // When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
  // Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
  Adafruit_NeoPixel NEOPixel;  // NEOPIXEL line definition
  int line_lng;
  int idx;
  int color_idx;
} neo_pixel_swipe_t;

// ------ VARIABLES ---------//

// ROS Node Handle
ros::NodeHandle  nh;

neo_pixel_swipe_t line_swipe[] = {
  { Adafruit_NeoPixel(NUMPIXELS, PIN_LEFT,  NEO_GRB + NEO_KHZ800), DEFAULT_LINE_LNG, 0, 0 },  // Left line definition
  { Adafruit_NeoPixel(NUMPIXELS, PIN_RIGHT, NEO_GRB + NEO_KHZ800), DEFAULT_LINE_LNG, 0, 0 }  // Right line definition
};
// Define size Adafruit_NeoPixel lines
#define SIZE_NEOPIXEL 2
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

// ------SUBSCRIBER --------//

// char tmp_buffer[50];
/**
   @brief TwistMessageCb Twist message callback and conversion linear and angular velocity in led effect
*/
void TwistMessageCb( const geometry_msgs::Twist& msg) {
  // Save linear velocity
  line_vel = msg.linear.x;
  line_ang = msg.angular.z;
  // LOG linear and angular
  // sprintf(tmp_buffer, "lin:%.2f ang:%.2f", msg.linear.x, msg.angular.z);
  // nh.loginfo(tmp_buffer);
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

  // This initializes the NeoPixel library.
  for (unsigned int pixIdx = 0; pixIdx < SIZE_NEOPIXEL; ++pixIdx) {
    line_swipe[pixIdx].NEOPixel.begin();
  }
  // Evaluate size of colors led array
  size_colors = sizeof(colors) / sizeof(uint32_t);

  Timer1.initialize(500000);             // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(callbackLeft);  // attaches callback() as a timer overflow interrupt

  Timer3.initialize(500000);              // initialize timer1, and set a 1/2 second period
  Timer3.attachInterrupt(callbackRight);  // attaches callback() as a timer overflow interrupt

  Timer1.stop();
  Timer3.stop();

  // Initialization ROS node
  nh.initNode();
  // Initialization subscriber
  nh.subscribe(sub);

  // Get param gain linear velocity
  if (!nh.getParam("~k_v", &k_v, 1)) {
    k_v = 1;
  }
  // Get param gain angular velocity
  if (!nh.getParam("~k_w", &k_w, 1)) {
    k_w = 1;
  }

  // Load default led configuration
  colors[0] = Adafruit_NeoPixel::Color(0, INTENSITY, 0);
  colors[1] = Adafruit_NeoPixel::Color(INTENSITY, 0, 0);
  colors[2] = Adafruit_NeoPixel::Color(0, 0, INTENSITY);
  colors[3] = Adafruit_NeoPixel::Color(INTENSITY, INTENSITY, 0);
  colors[4] = Adafruit_NeoPixel::Color(INTENSITY, 0, INTENSITY);
}

// --------LOOP ----------//

volatile float vel_left = line_vel;
volatile float vel_right = line_vel;
volatile float vel_left_old = vel_left;
volatile float vel_right_old = vel_right;

void loop() {
  //digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led

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
    led_swipe2(&line_swipe[0], colors[line_swipe[0].color_idx], L1_POS, L2_POS);
    // Update index swipe led
    update_index(&line_swipe[0], vel_left);
  }
}
/**
   @brief Timer callback to update velocity for neo pixel line right
*/
void callbackRight() {
  if (vel_right != 0) {
    // Launch swipe effect with the color on the list
    led_swipe2(&line_swipe[1], colors[line_swipe[1].color_idx], L1_POS, L2_POS);
    // Update index swipe led
    update_index(&line_swipe[1], vel_right);
  }
}
/**
   Update index
*/
void update_index(neo_pixel_swipe_t *line, float vel) {
  // Set increase or decrease velocity led
  line->idx += sign(vel);
  // Reset index after
  if (line->idx >= L2_POS) {
    line->idx = L1_POS;
    line->color_idx++;
  } else if (line->idx <= L1_POS) {
    line->idx = L2_POS;
    line->color_idx--;
  }
  // Check color index
  if (line->color_idx >= 5) {
    line->color_idx = 0;
  } else if (line->color_idx < 0) {
    line->color_idx = 4;
  }
}
/**
   @brief Swipe led with a short colored line with length lng
   @param pixels Pointer to Adafruit_NeoPixel
   @param pixel_size length of pixels
   @param color The color to set the line
   @param idx index to plot the swipe
   @param sta first neopixel led to start
   @param sto last neopixel to stop the effect
*/
void led_swipe2(neo_pixel_swipe_t *line, uint32_t color, int sta, int sto) {
  for (int i = sta - line->line_lng; i < sto; ++i) {
    // Set i led to the color
    if (line->idx >= i && line->idx < i + line->line_lng) {
      line->NEOPixel.setPixelColor(i, color);
    } else {
      // Otherwise switch off the i led
      line->NEOPixel.setPixelColor(i, 0);
    }
  }
  // This sends the updated pixel color to the hardware.
  line->NEOPixel.show();
}
/**
  @brief Swipe led along the neopixel line in order from the first led to the last
  @param pixels Pointer to Adafruit_NeoPixel
  @param pixel_size length of pixels
  @param color The color to set the line
  @param d delay to show the effect
  @param sta first neopixel led to start
  @param sto last neopixel to stop the effect
*/
void led_swipe(Adafruit_NeoPixel * pixels, size_t pixel_size, uint32_t color, uint32_t d, uint32_t sta, uint32_t sto) {
  for (unsigned int i = sta; i < sto; i++) {
    for (unsigned int pixIdx = 0; pixIdx < pixel_size; ++pixIdx) {
      pixels[pixIdx].setPixelColor(i, color);
      pixels[pixIdx].show(); // This sends the updated pixel color to the hardware.
    }
    delay(d); // Delay for a period of time (in milliseconds).
  }
  // Delay for a period of time (in milliseconds).
  delay(500);
}
/**
   @brief Swipe led along the neopixel line in order from the last led to the first
   @param pixels Pointer to Adafruit_NeoPixel
   @param pixel_size length of pixels
   @param color The color to set the line
   @param d delay to show the effect
   @param sta first neopixel led to start
   @param sto last neopixel to stop the effect
*/
void led_swipe_reverse(Adafruit_NeoPixel * pixels, size_t pixel_size, uint32_t color, uint32_t d, uint32_t sta, uint32_t sto) {
  for (unsigned int i = sta; i < sto; i++) {
    for (unsigned int pixIdx = 0; pixIdx < pixel_size; ++pixIdx) {
      pixels[pixIdx].setPixelColor(i, color);
      pixels[pixIdx].show(); // This sends the updated pixel color to the hardware.
    }
    delay(d); // Delay for a period of time (in milliseconds).
  }
  // Delay for a period of time (in milliseconds).
  delay(300);
}

int sign(float x) {
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}

