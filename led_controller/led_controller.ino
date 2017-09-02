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
// delay for half a second
#define DELAY_VAL          20

// Topic definition
#define TWIST_SUBSCRIBER_TOPIC "/cmd_vel"
// -------------------------//

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

#include "TimerOne.h"
// ROS header
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define L1_POS             10
#define L2_POS             NUMPIXELS-10
// pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
#define NEOPIXEL_LED_OFF Adafruit_NeoPixel::Color(0, 0, 0)       // Default LED OFF
#define NEOPIXEL_LED_ON Adafruit_NeoPixel::Color(255, 255, 255)  // Default WHITE ON

// ------ VARIABLES ---------//

// ROS Node Handle
ros::NodeHandle  nh;
// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
Adafruit_NeoPixel NEOPixel[] = {
  Adafruit_NeoPixel(NUMPIXELS, PIN_LEFT,  NEO_GRB + NEO_KHZ800),  // NEOPIXEL left line definition
  Adafruit_NeoPixel(NUMPIXELS, PIN_RIGHT, NEO_GRB + NEO_KHZ800)   // NEOPIXEL right line definition
  };
// Define size Adafruit_NeoPixel lines
#define SIZE_NEOPIXEL 2

// TODO To improve and update from ROS param list
// LED colors sequence
#define INTENSITY          50
uint32_t colors[] = {
  Adafruit_NeoPixel::Color(0, INTENSITY, 0),
  Adafruit_NeoPixel::Color(INTENSITY, 0, 0),
  Adafruit_NeoPixel::Color(0, 0, INTENSITY),
  Adafruit_NeoPixel::Color(INTENSITY, INTENSITY, 0),
  Adafruit_NeoPixel::Color(INTENSITY, 0, INTENSITY),
  Adafruit_NeoPixel::Color(0, INTENSITY, INTENSITY),
  Adafruit_NeoPixel::Color(INTENSITY, INTENSITY, INTENSITY)
};
// Size of LED colors
int size_colors = 0;

// ------SUBSCRIBER --------//

/**
 * @brief TwistMessageCb Twist message callback and conversion linear and angular velocity in led effect
 */
void TwistMessageCb( const geometry_msgs::Twist& msg) {
  digitalWrite(LED_BUILTIN, HIGH - digitalRead(LED_BUILTIN)); // blink the led
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
  for(unsigned int pixIdx = 0; pixIdx < SIZE_NEOPIXEL; ++pixIdx) {
    NEOPixel[pixIdx].begin(); 
  }
  // Evaluate size of colors led array
  size_colors = sizeof(colors) / sizeof(uint32_t);

  Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  //Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt

  // Initialization ROS node
  nh.initNode();
  // Initialization subscriber
  nh.subscribe(sub);

  // TODO use to read list of color sequence
  /*
  float pid_constants[3];
  if (!nh.getParam("~pid", pid_constants, 3)) {
    //default values
    pid_constants[0] = 1;
    pid_constants[1] = 0.24;
    pid_constants[2] = 0.01;
  }
  */
}

// --------LOOP ----------//

void loop() {
  // Color swipe led loop
  for (int i = 0; i < size_colors; ++i) {
     led_swipe2(NEOPixel, SIZE_NEOPIXEL, colors[i], 100, 0, NUMPIXELS, 10);
  }
}

// -------------------------//
/**
 * @brief Timer callback with spin once and update of the params
 */
void callback() {
  //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
  nh.spinOnce();
}
/**
 * @brief Swipe led with a short colored line with length lng
 * @param pixels Pointer to Adafruit_NeoPixel
 * @param pixel_size length of pixels
 * @param color The color to set the line
 * @param d delay to show the effect
 * @param sta first neopixel led to start
 * @param sto last neopixel to stop the effect
 * @param lng the length of the line
 */
void led_swipe2(Adafruit_NeoPixel *pixels, size_t pixel_size, uint32_t color, uint32_t d, uint32_t sta, uint32_t sto, uint32_t lng) {
  for (unsigned int idx = sta; idx < sto; idx++) {
    for (unsigned int i = sta; i < sto; i++) {
      // Set i led to the color
      for(unsigned int pixIdx = 0; pixIdx < pixel_size; ++pixIdx) {
        if(i >= idx && i < idx + lng) {
          pixels[pixIdx].setPixelColor(i, color);
        } else {
          // Otherwise switch off the i led
          pixels[pixIdx].setPixelColor(i, 0);
        }
      }
    }
    for(unsigned int pixIdx = 0; pixIdx < pixel_size; ++pixIdx) {
      pixels[pixIdx].show(); // This sends the updated pixel color to the hardware.
    }
    delay(d); // Delay for a period of time (in milliseconds).
  }
}
/**
 * @brief Swipe led along the neopixel line in order from the first led to the last
 * @param pixels Pointer to Adafruit_NeoPixel
 * @param pixel_size length of pixels
 * @param color The color to set the line
 * @param d delay to show the effect
 * @param sta first neopixel led to start
 * @param sto last neopixel to stop the effect
 */
void led_swipe(Adafruit_NeoPixel *pixels, size_t pixel_size, uint32_t color, uint32_t d, uint32_t sta, uint32_t sto) {
  for (unsigned int i = sta; i < sto; i++) {
    for(unsigned int pixIdx = 0; pixIdx < pixel_size; ++pixIdx) {
      pixels[pixIdx].setPixelColor(i, color);
      pixels[pixIdx].show(); // This sends the updated pixel color to the hardware.
    }
    delay(d); // Delay for a period of time (in milliseconds).
  }
  // Delay for a period of time (in milliseconds).
  delay(500);
}
/**
 * @brief Swipe led along the neopixel line in order from the last led to the first
 * @param pixels Pointer to Adafruit_NeoPixel
 * @param pixel_size length of pixels
 * @param color The color to set the line
 * @param d delay to show the effect
 * @param sta first neopixel led to start
 * @param sto last neopixel to stop the effect
 */
void led_swipe_reverse(Adafruit_NeoPixel *pixels, size_t pixel_size, uint32_t color, uint32_t d, uint32_t sta, uint32_t sto) {
  for (unsigned int i = sta; i < sto; i++) {
    for(unsigned int pixIdx = 0; pixIdx < pixel_size; ++pixIdx) {
      pixels[pixIdx].setPixelColor(i, color);
      pixels[pixIdx].show(); // This sends the updated pixel color to the hardware.
    }
    delay(d); // Delay for a period of time (in milliseconds).
  }
  // Delay for a period of time (in milliseconds).
  delay(300);
}
