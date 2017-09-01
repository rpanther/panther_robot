// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

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

#define L1_POS             10
#define L2_POS             NUMPIXELS-10

#define INTENSITY          50

// -------------------------//


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include "TimerOne.h"
// ROS header
#include <ros.h>
#include <geometry_msgs/Twist.h>

// TODO To remove
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels_left = Adafruit_NeoPixel(NUMPIXELS, PIN_LEFT, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels_right = Adafruit_NeoPixel(NUMPIXELS, PIN_RIGHT, NEO_GRB + NEO_KHZ800);

// pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
uint32_t color = pixels_left.Color(255, 255, 255);
uint32_t led_off = pixels_left.Color(0,0,0);

// LED colors sequence
uint32_t colors[] = {
  pixels_left.Color(0, INTENSITY, 0),
  pixels_left.Color(INTENSITY, 0, 0),
  pixels_left.Color(0, 0, INTENSITY),
  pixels_left.Color(INTENSITY, INTENSITY, 0),
  pixels_left.Color(INTENSITY, 0, INTENSITY),
  pixels_left.Color(0, INTENSITY, INTENSITY),
  pixels_left.Color(INTENSITY, INTENSITY, INTENSITY),
  pixels_left.Color(0, 0, 0),
  };
// Size of LED colors
int size_colors = 0;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup() {
  // Initialization LED builtin
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialization Green strip LED
  pinMode(LED_GREEN, OUTPUT);
  // Set low LED
  digitalWrite(LED_GREEN, LOW);
  pixels_left.begin(); // This initializes the NeoPixel library.
  pixels_right.begin(); // This initializes the NeoPixel library.
  // Evaluate size of colors led array
  size_colors = sizeof(colors)/sizeof(uint32_t);

  Timer1.initialize(500000);         // initialize timer1, and set a 1/2 second period
  Timer1.pwm(9, 512);                // setup pwm on pin 9, 50% duty cycle
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  
  // Initialization ROS node
  nh.initNode();
  // Initialization subscriber
  nh.subscribe(sub);
  // TODO to REMOVE
  nh.advertise(chatter);
  // -- END

  // TODO use to read list of color sequence
  float pid_constants[3];
  if(!nh.getParam("~pid", pid_constants, 3)){ 
       //default values
       pid_constants[0]= 1;
       pid_constants[1]=0.24;
       pid_constants[2]=0.01; 
  }
}
// ROS spin update
void callback() {
  //digitalWrite(LED_BUILTIN, HIGH-digitalRead(LED_BUILTIN));   // blink the led
  nh.spinOnce();
}

void loop() {
  //TODO To REMOVE
  str_msg.data = hello;
  chatter.publish( &str_msg );
  // -- END
  
  // Color swipe led loop
  for(int i = 0; i < size_colors; ++i) {
    led_swipe(colors[i], DELAY_VAL, 0, NUMPIXELS);
  }
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
