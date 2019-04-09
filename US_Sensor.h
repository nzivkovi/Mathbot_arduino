#ifndef Sensor_h
#define Sensor_h

//#define NB_SAMPLE 25

#include "Arduino.h"
#include "Math.h"

struct Sensor
{
  Sensor(int trigPin, int echoPin);
  int trigPin, echoPin;

  //helpers
  /*int find_median(int* measurements);
  int partition(int *arr, int left, int right);
  int select(int *arr, int left, int right, int i);*/

  //distance
  int get_distance();
};

#endif
