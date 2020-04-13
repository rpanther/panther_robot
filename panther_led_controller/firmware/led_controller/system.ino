/*
typedef struct _soft_timer {
  bool start;
  unsigned long currentMillis;
  unsigned long previousMillis;
  long interval;
} soft_timer_t;
*/

void soft_timer_init(soft_timer_t &timer, float inter) {
  timer.start = true;
  timer.currentMillis = 0;
  timer.previousMillis = 0;
  timer.interval = inter * 1000;
}

void soft_timer_start(soft_timer_t &timer) {
  timer.start = true;
  timer.previousMillis = millis();
}

void soft_timer_stop(soft_timer_t &timer) {
  timer.start = false;
}

bool soft_timer_run(soft_timer_t &timer) {
  if(timer.start) {
    // software timer
    timer.currentMillis = millis();
    // Evaluate interval
    if (timer.currentMillis - timer.previousMillis >= timer.interval) {
      // save the last time
      timer.previousMillis = timer.currentMillis;
  
      return true;
    }
  }
  return false;
}

int sign(float x) {
  if (x > 0) return 1;
  else if (x < 0) return -1;
  else return 0;
}
