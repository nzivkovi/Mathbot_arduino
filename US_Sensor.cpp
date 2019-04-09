#include "US_Sensor.h"

Sensor::Sensor(int trigPin, int echoPin)
{
  this->trigPin = trigPin;
  this->echoPin = echoPin;
  // Define echoPin as Input
  pinMode(echoPin, INPUT);
  // Define trigPin as Output
  pinMode(trigPin, OUTPUT);
}

int Sensor::get_distance()
{
  //max time in microseconds to wait for the pulse to be completed
  unsigned long max_delay_time = 2500;
  int duration;
  //clears the trigPin
  //digitalWrite(this->trigPin, LOW);
  //delayMicroseconds(2);
  digitalWrite(this->trigPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(this->trigPin, LOW);
  duration = pulseIn(this->echoPin, HIGH, max_delay_time);
  if (duration == 0){
    return 30;  
  } else {
    return int((duration / 2) / 29.0);    
  }
}

/*int Sensor::get_distance()
{
  int arr[NB_SAMPLE];
  //clears the trigPin
  digitalWrite(this->trigPin, LOW);
  delayMicroseconds(2);
  int duration;
  for(int i=0; i<25; i++)
  {
    digitalWrite(this->trigPin, HIGH);
    delayMicroseconds (10);
    digitalWrite(this->trigPin, LOW);
    duration = pulseIn(this->echoPin, HIGH);
    arr[i] = int(duration * 0.5 * 0.0344);
  }
  return find_median(arr);
}

int Sensor::find_median(int* measurements)
{
  return this->select(measurements, 0, 24, 12);
}

int Sensor::partition(int *arr, int left, int right)
{
  int x = arr[left];
  int i = left;
  int key;
  for (int j = left + 1; j <= right; j++)
  {
    if (arr[j] <= x)
    { 
      i = i+1;
      key = arr[i];
      arr[i] = arr[j];
      arr[j] = key;     
    }
  }
  key = arr[i];
  arr[i]= arr[left];
  arr[left] = key;
  return i;
}

int Sensor::select(int *arr, int left, int right, int i)
{
  if (left == right)
  {
    return arr[left];
  }
  int r = this->partition(arr, left, right);
  int k = r - left + 1;
  if (i == k)
  {
    return arr[r];
  }
  if (i < k)
  {
    return this->select(arr, left, r - 1, i);
  }
  else
   {
    return select(arr, r + 1, right, i - k);
  }
}*/
