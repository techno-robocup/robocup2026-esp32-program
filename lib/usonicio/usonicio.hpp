#pragma once
#include <Arduino.h>

class UltrasonicIO {
 public:
  UltrasonicIO(int _trig, int _echo);
  void readUsonic(long*);

 private:
  int trig, echo;
};
