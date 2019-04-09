#include "AngularSpeedSupervisor.h"

AngularSpeedSupervisor::AngularSpeedSupervisor(Encoder* enc, Motor* motor, PIDController* pid)
{
  this->enc = enc;
  this->motor = motor;
  this->pid = pid;

  this->ref_angular_speed = 0.0;
  this->last_angle = 0.0;
}

double AngularSpeedSupervisor::get_error(double dt)
{
  double current_angle = this->enc->count * 2 * M_PI / this->enc->cpr;
  double current_angular_speed = (current_angle - last_angle) / dt;
  this->last_angle = current_angle;
  /*Serial.print(current_angular_speed);
  Serial.print(" ");
  Serial.print(this->ref_angular_speed);
  Serial.print(" ");*/
  return this->ref_angular_speed - current_angular_speed;
}

void AngularSpeedSupervisor::execute(double dt)
{
  if(this->ref_angular_speed != 0)
  {
    double u = this->pid->execute(this->get_error(dt), dt);
    //Serial.print(u);
    //Serial.print(" ");
    if(u >= 0)
    {
      u = map(u, 0, this->motor->max_speed, 0 , 255);
      this->motor->set_speed(u);
      //this->motor->set_speed(255);
    }
    else
    {
      u = map(u, -this->motor->max_speed, 0, -255, 0);
      this->motor->set_speed(u);
      //this->motor->set_speed(255);
    }
  }
}

void AngularSpeedSupervisor::set_ref_by_speed(int speed, bool reset_pid)
{
  this->ref_angular_speed = speed / 1000.0;
  if(this->ref_angular_speed == 0.0)
  {
    this->motor->stop();
  }
  if(reset_pid)
  {
    this->pid->reset_errors();
  }
}



