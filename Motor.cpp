#include "Motor.h"

Motor::Motor(int pinF, int pinB, int lower_pwm, int max_rpm)
{
  this->pinF = pinF;
  this->pinB = pinB;
  this->lower_pwm = lower_pwm;
  this->max_speed = 2 * PI * max_rpm / 60;

  pinMode(this->pinF, OUTPUT);
  pinMode(this->pinB, OUTPUT);
}

int Motor::ensure_duty_cycle(int duty_cycle)
{
  if(duty_cycle > 255)
  {
    duty_cycle = 255;
  }
  
  if(duty_cycle < 0)
  {
    duty_cycle = 0;
  }

  return duty_cycle;
}

void Motor::set_speed_forward(int duty_cycle)
{
  analogWrite(this->pinB, 0);
  int safe_duty_cycle = map(this->ensure_duty_cycle(duty_cycle), 0, 255, this->lower_pwm, 255);
  analogWrite(this->pinF, safe_duty_cycle);
}

void Motor::set_speed_backward(int duty_cycle)
{
  analogWrite(this->pinF, 0);
  int safe_duty_cycle = map(this->ensure_duty_cycle(duty_cycle), 0, 255, this->lower_pwm, 255);
  analogWrite(this->pinB, safe_duty_cycle);
}

void Motor::set_speed(int speed)
{
  if(speed >= 0)
  {
    this->set_speed_forward(speed);
  }
  else
  {
    this->set_speed_backward(-speed);
  }
}

void Motor::stop()
{
  analogWrite(this->pinF, 0);
  analogWrite(this->pinB, 0);
}

