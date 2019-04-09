#ifndef Encoder_h
#define Encoder_h

#include "Arduino.h"

struct Encoder
{
  Encoder(int pinA, int pinB, unsigned int cpr);

  int pinA, pinB;
  //cycles per revolution
  unsigned int cpr;

  volatile long count;
  long count_old;

  long A_old;
  long B_new;

  void A_interrupt();
  void B_interrupt();
};

#endif
