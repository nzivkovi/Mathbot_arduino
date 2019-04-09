#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

struct Motor
{
  Motor(int pinF, int pinB, int lower_pwm, int max_rpm);
  int pinF, pinB, lower_pwm;
  double max_speed;

  //helpers
  int ensure_duty_cycle(int duty_cycle);

  //speed
  void set_speed_forward(int duty_cycle);
  void set_speed_backward(int duty_cycle);
  void set_speed(int speed);
  void stop();
};

#endif
