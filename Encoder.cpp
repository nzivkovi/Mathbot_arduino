#include "Encoder.h"

Encoder::Encoder(int pinA, int pinB, unsigned int cpr)
{
  this->pinA = pinA;
  this->pinB = pinB;
  this->cpr = cpr;

  this->count = 0;
  this->count_old = count;

  pinMode(this->pinA, INPUT);
  pinMode(this->pinB, INPUT);
}

void Encoder::A_interrupt()
{
  this->B_new == this->A_old ? this->count++ : this->count--;
  this->A_old = digitalRead(this->pinA);
  //Serial.println(this->count);
}

void Encoder::B_interrupt()
{
  this->B_new = digitalRead(this->pinB);
  this->B_new == this->A_old ? this->count++ : this->count--;
  //Serial.println(this->count);
}

