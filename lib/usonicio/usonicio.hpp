#pragma once
#include <Arduino.h>

class UltrasonicIO {
 public:
  UltrasonicIO(int _trig, int _echo);
  void read(long*);

 private:
  int trig, echo;
};
