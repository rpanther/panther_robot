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

#include <Wire.h>
#include "SFR10.hpp"

SFR10::SFR10(int srfAddress, String name)
{
    // I2C address
    srfAddress_ = srfAddress;
    name_ = &name;
}

int SFR10::getRange()
{
  int range = 0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(srfAddress_); // transmit to device #112 (0x70)
  // the address specified in the datasheet is 224 (0xE0)
  // but i2c adressing uses the high 7 bits so it's 112
  Wire.write(byte(0x00));             // sets register pointer to the command register (0x00)
  Wire.write(byte(0x51));             // command sensor to measure
  // - 0x50 for inches
  // - 0x51 for centimeters
  // - 0x52 for ping microseconds
  Wire.endTransmission();             // stop transmitting
  // step 2: wait for readings to happen
  delay(70);                          // datasheet suggests at least 65 milliseconds
  // step 3: instruct sensor to return a particular echo reading
  Wire.beginTransmission(srfAddress_); // transmit to device #112
  Wire.write(byte(0x02));             // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();             // stop transmitting
  // step 4: request reading from sensor
  Wire.requestFrom(srfAddress_, 2);    // request 2 bytes from slave device #112
  // step 5: receive reading from sensor
  if (2 <= Wire.available()) {        // if two bytes were received
    range = Wire.read();              // receive high byte (overwrites previous reading)
    range = range << 8;               // shift high byte to be high 8 bits
    range |= Wire.read();             // receive low byte as lower 8 bits
  }
  return range;
}