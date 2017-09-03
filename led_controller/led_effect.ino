/*
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
*/

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
