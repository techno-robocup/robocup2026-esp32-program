#pragma once
#ifndef __ROBOT__MOTORIO__HPP
#define __ROBOT__MOTORIO__HPP 1
#include <Arduino.h>
class MOTORIO {
 public:
  MOTORIO();
  MOTORIO(const std::int8_t&, const int&);
  MOTORIO& operator=(const MOTORIO&) = default;
  virtual void run_msec(const int&);

 private:
  std::int8_t PIN;
  unsigned long prev_msec;  // Changed to unsigned for overflow handling
  int interval;
};
#endif
