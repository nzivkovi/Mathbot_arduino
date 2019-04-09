#include "PID.h"

PIDController::PIDController(double k_p, double k_i, double k_d)
{
  this->k_p = k_p;
  this->k_i = k_i;
  this->k_d = k_d;

  this->E = 0.0;
  this->e_old = 0.0;
}

void PIDController::reset_errors()
{
  this->E = 0.0;
  this->e_old = 0.0;
}

double PIDController::execute(double error, double dt)
{
  this-> E += error * dt;
  double e_dot = (error - e_old) / dt;
  double u = (this->k_p * error + this->k_i * this->E + this->k_d * e_dot) / 100;
  this->e_old = error;
  return u;
}

