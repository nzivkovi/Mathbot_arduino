#ifndef PID_h
#define PID_h

#include "Arduino.h"

struct PIDController
{
  PIDController(double k_p, double k_i, double k_d);

  //params
  double k_p, k_i, k_d;

  //errors
  double E, e_old;
  void reset_errors();

  //u
  double execute(double error, double dt);
};

#endif
