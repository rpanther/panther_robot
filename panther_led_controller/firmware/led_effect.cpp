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

#include "led_effect.hpp"

int sign(float x) {
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}

void led_swipe_init(neo_pixel_swipe_t &line, Adafruit_NeoPixel *neopixel, int start, int stop, int length, uint32_t bg_color) {
  // Initialize start, stop and background effect
  line.NEOPixel = neopixel;
  line.start = start;
  line.stop = stop;
  line.bg_color = bg_color;
  line.length = length;
  // Initialize index
  line.idx = line.start;

  // Initialize neopixel
  line.NEOPixel->begin();
  // Reset line
  led_swipe_reset(line);
}

void led_swipe_reset(neo_pixel_swipe_t &line) {
  // reset index
  line.idx = line.start;
  // Reset line
  for (int i = line.start; i < line.stop; ++i) {
    line.NEOPixel->setPixelColor(i, line.bg_color);
  }
  // This sends the updated pixel color to the hardware.
  line.NEOPixel->show();
}

void led_swipe_set(neo_pixel_swipe_t &line, uint32_t bg_color) {
  // reset index
  line.idx = line.start;
  // Reset line
  for (int i = line.start; i < line.stop; ++i) {
    line.NEOPixel->setPixelColor(i, bg_color);
  }
  // This sends the updated pixel color to the hardware.
  line.NEOPixel->show();
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
int led_swipe_run(neo_pixel_swipe_t &line, uint32_t color, float vel) {
  for (int i = line.start - line.length; i < line.stop; ++i) {
    // Set i led to the color
    if (line.idx >= i && line.idx < i + line.length) {
      line.NEOPixel->setPixelColor(i, color);
    } else {
      // Otherwise switch off the i led
      line.NEOPixel->setPixelColor(i, line.bg_color);
    }
  }
  // This sends the updated pixel color to the hardware.
  line.NEOPixel->show();

  // Set increase or decrease velocity led
  line.idx += sign(vel);
  // Reset index after
  if (line.idx >= line.stop) {
    line.idx = line.start;
    return 1;
  } else if (line.idx <= line.start) {
    line.idx = line.stop;
    return -1;
  }
  return 0;
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
