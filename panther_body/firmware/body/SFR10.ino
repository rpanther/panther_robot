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
 * @brief Connect I2C
 * SFR10: https://www.robot-electronics.co.uk/htm/srf10tech.htm
 */
void SFR10_connect(sensor_msgs::Range* range_msg)
{
  // Waits to make sure everything is powered up before sending or receiving data
  Wire.begin();
  // Initialize range sensor
  range_msg->radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg->field_of_view = 1;
  range_msg->min_range = 0.0;
  range_msg->max_range = 5.0;
}
/**
 * @brief Initialize ROS
 */
void SFR10_init(ros::NodeHandle* nh, SFR10_t &SFR10)
{
  // Initialize topic
  nh->advertise(*SFR10.pub); 
}
/**
 * @brief Publish range sensor
 */
void SFR10_publish(ros::NodeHandle* nh, SFR10_t &SFR10)
{
  float range = ((float) SFR10.distance) / 100.0;
  // Publish message
  SFR10.msg->header.stamp = nh->now();
  SFR10.msg->header.frame_id = SFR10.frame_id;
  SFR10.msg->range = range;
  SFR10.pub->publish(SFR10.msg);
}
/**
 * @brief Update SFR10 status
 */
void SFR10_update(SFR10_t &SFR10)
{
  // read range sensor
  SFR10.distance = SFR10_getRange(SFR10);
}
/**
 * @brief SFR10 sonar
 * reference: https://www.robot-electronics.co.uk/htm/srf10tech.htm
 */
int SFR10_getRange(SFR10_t &SFR10)
{
  int distance = 0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(SFR10.address); // transmit to device #112 (0x70)
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
  Wire.beginTransmission(SFR10.address); // transmit to device #112
  Wire.write(byte(0x02));             // sets register pointer to echo #1 register (0x02)
  Wire.endTransmission();             // stop transmitting
  // step 4: request reading from sensor
  Wire.requestFrom(SFR10.address, 2);    // request 2 bytes from slave device #112
  // step 5: receive reading from sensor
  if (2 <= Wire.available()) {        // if two bytes were received
    distance = Wire.read();           // receive high byte (overwrites previous reading)
    distance = distance << 8;         // shift high byte to be high 8 bits
    distance |= Wire.read();          // receive low byte as lower 8 bits
  }
  return distance;
}
