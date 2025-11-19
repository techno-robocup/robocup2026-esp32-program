#pragma once
#ifndef __ROBOT__ARMIO__HPP
#define __ROBOT__ARMIO__HPP 1
#include <Arduino.h>

class ARMIO {
 public:
  ARMIO();
  ARMIO(const std::int8_t& arm_pulse, const std::int8_t& arm_feedback, const std::int8_t& wire_sig);
  ARMIO& operator=(const ARMIO&) = default;

  void init_pwm();
  void arm_set_position(const int& position);
  void wire_tension_function(const bool& enable);
  void updatePID();

 private:
  std::int8_t arm_pulse_pin;
  std::int8_t arm_feedback_pin;
  std::int8_t wire_sig_pin;

  unsigned long prev_msec;
  unsigned long wire_prev_msec;
  int servo_interval;

  // PID controller variables
  float kp;
  float ki;
  float kd;
  float previous_error;
  float integral;
  int target_position;

  // Read current arm position from feedback pin
  int getCurrentPosition();
};

#endif
