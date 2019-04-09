#ifndef AngularSpeedSupervisor_h
#define AngularSpeedSupervisor_h

#include "math.h"

#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

struct AngularSpeedSupervisor
{
  AngularSpeedSupervisor(Encoder* enc, Motor* motor, PIDController* pid);

  //members
  Encoder* enc;
  Motor* motor;
  PIDController* pid;

  //control
  double get_error(double dt);
  void execute(double dt);

  //double
  volatile double ref_angular_speed;
  double last_angle;
  void set_ref_by_speed(int speed, bool reset_pid);
};

#endif
