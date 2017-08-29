// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

// Led definition
#define PIN_LEFT            2
#define PIN_RIGHT           3
#define LED_GREEN          10
// NeoPixels attached
#define NUMPIXELS          43
// delay for half a second
#define DELAY_VAL          20

#define L1_POS             10
#define L2_POS             NUMPIXELS-10

#define INTENSITY          50

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels_left = Adafruit_NeoPixel(NUMPIXELS, PIN_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_right = Adafruit_NeoPixel(NUMPIXELS, PIN_RIGHT, NEO_GRB + NEO_KHZ800);

// pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
uint32_t color = pixels_left.Color(255, 255, 255);
uint32_t led_off = pixels_left.Color(0,0,0);

void setup() {
  // Initialization Green strip LED
  pinMode(LED_GREEN, OUTPUT);
  // Set low LED
  digitalWrite(LED_GREEN, LOW);
  pixels_left.begin(); // This initializes the NeoPixel library.
  pixels_right.begin(); // This initializes the NeoPixel library.

  nh.initNode();
  nh.advertise(chatter);
}

void loop() {

  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  
  led_swipe(pixels_left.Color(0, INTENSITY, 0), DELAY_VAL, 0, NUMPIXELS);
  led_swipe(pixels_left.Color(INTENSITY, 0, 0), DELAY_VAL, 0, NUMPIXELS);
  led_swipe(pixels_left.Color(0, 0, INTENSITY), DELAY_VAL, 0, NUMPIXELS);
  led_swipe(pixels_left.Color(INTENSITY, INTENSITY, 0), DELAY_VAL, 0, NUMPIXELS);
  led_swipe(pixels_left.Color(INTENSITY, 0, INTENSITY), DELAY_VAL, 0, NUMPIXELS);
  led_swipe(pixels_left.Color(0, INTENSITY, INTENSITY), DELAY_VAL, 0, NUMPIXELS);
  led_swipe(pixels_left.Color(INTENSITY, INTENSITY, INTENSITY), DELAY_VAL, 0, NUMPIXELS);
}

void led_swipe(uint32_t color, uint32_t d, uint32_t sta, uint32_t sto) {
  for(unsigned int i=sta; i<sto; i++){
    pixels_left.setPixelColor(i, color);
    pixels_left.show(); // This sends the updated pixel color to the hardware.
    pixels_right.setPixelColor(i, color);
    pixels_right.show(); // This sends the updated pixel color to the hardware.
    delay(d); // Delay for a period of time (in milliseconds).
  }
  // Delay for a period of time (in milliseconds).
  delay(500); 
}

void led_swipe_reverse(uint32_t color, uint32_t d, uint32_t sta, uint32_t sto) {
  for(unsigned int i=sta; i<sto; i++){
    pixels_left.setPixelColor(sto-i, color);
    pixels_left.show(); // This sends the updated pixel color to the hardware.
    pixels_right.setPixelColor(sto-i, color);
    pixels_right.show(); // This sends the updated pixel color to the hardware.
    delay(d); // Delay for a period of time (in milliseconds).
  }
  // Delay for a period of time (in milliseconds).
  delay(300); 
}
