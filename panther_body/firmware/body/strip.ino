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

/**
 * @brief Initialize Neopixel
 */
void NEOpixel_init(neo_pixel_t &strip)
{
  pinMode(strip.NEOPixel.getPin(), OUTPUT);
  // Initialize STRIPs
  strip.NEOPixel.begin();
  strip.NEOPixel.setBrightness(strip.brightness);
  strip.NEOPixel.show(); // Initialize all pixels to 'off'
  // Initialize effects
  strip.range.enable = false;
  strip.bright.enable = false;
  strip.bright.step = 10;
}
void NEOpixel_clear(neo_pixel_t &strip)
{
  strip.NEOPixel.clear();
}
void NEOpixel_update(neo_pixel_t &strip)
{
  // Update rainbow
  NEOpixel_rainbow_update(strip);
  // Update brightness effect
  NEOpixel_bright_update(strip);
  // Show
  strip.NEOPixel.show();
}
/**
 * @brief Set a range color
 */
void NEOpixel_range(neo_pixel_t &strip, uint32_t color, int range, bool reverse)
{
  // Set color
  strip.range.color = color;
  // set position
  strip.range.start = 0;
  strip.range.end = range;
  if(range > strip.NEOPixel.numPixels())
  {
    strip.range.end = strip.NEOPixel.numPixels();
  }
  if(reverse)
  {
    strip.range.start = strip.NEOPixel.numPixels() - range;
    strip.range.end = strip.NEOPixel.numPixels();
  }
  // Update colors
  for(unsigned int i = strip.range.start; i < strip.range.end; ++i)
  {
    strip.NEOPixel.setPixelColor(i, strip.range.color);
  }
  // Enable effect
  //strip.range.enable = true;
}
/**
 * @brief Stop range effect
 */
void NEOpixel_range_stop(neo_pixel_t &strip)
{
  strip.range.enable = false;
  strip.NEOPixel.clear();
}
/**
 * @brief Bright strip
 */
void NEOpixel_bright(neo_pixel_t &strip, uint32_t color, int brightness)
{
  if(strip.bright.enable)
    return;
  strip.bright.color = color;
  strip.bright.counter = strip.brightness;
  strip.bright.newbrightness = brightness;
  strip.bright.enable = true;
  strip.bright.status = true;
}
/**
 * @brief bright stop
 */
void NEOpixel_bright_stop(neo_pixel_t &strip)
{
  strip.NEOPixel.setBrightness(strip.brightness);
  strip.bright.enable = false;
}
/**
 * @brief brightness update
 */
void NEOpixel_bright_update(neo_pixel_t &strip)
{
  if(strip.bright.enable)
  {
    for(unsigned int i = 0; i < strip.NEOPixel.numPixels(); ++i)
    {
      strip.NEOPixel.setPixelColor(i, strip.bright.color);
    }
    if(strip.bright.status)
    {
      // Update counter effect
      strip.bright.counter += strip.bright.step;
      if(strip.bright.counter > strip.bright.newbrightness)
      {
        strip.bright.counter = strip.bright.newbrightness;
        strip.bright.status = false;
      }
    }
    else
    {
      strip.bright.counter -= strip.bright.step;
      if(strip.bright.counter < 0)
      {
        strip.bright.counter = 0;
        strip.bright.status = true;
      }
    }
    // Update brighness
    strip.NEOPixel.setBrightness(strip.bright.counter);
  }
}
/**
 * @brief Rainbow effect
 */
void NEOpixel_rainbow(neo_pixel_t &strip)
{
  if(strip.rainbow.enable)
    return;
  strip.rainbow.counter = 0;
  strip.rainbow.enable = true;
}
/**
 * @brief disable rainbow
 */
void NEOpixel_rainbow_stop(neo_pixel_t &strip)
{
  strip.rainbow.enable = false;
}
void NEOpixel_rainbow_update(neo_pixel_t &strip)
{
  if(strip.rainbow.enable)
  {
    for(unsigned int i = 0; i < strip.NEOPixel.numPixels(); i++) {
      strip.NEOPixel.setPixelColor(i, Wheel(&strip.NEOPixel, (i + strip.rainbow.counter) & 255));
    }
    //Increase counter
    strip.rainbow.counter += 1;
    if(strip.rainbow.counter > 255)
    {
      strip.rainbow.counter = 0;
    }
  }
}
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(Adafruit_NeoPixel* strip, byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip->Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip->Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip->Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
