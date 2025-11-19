#pragma once
#ifndef __ROBOT__ARMIO__HPP
#define __ROBOT__ARMIO__HPP 1
#include <Arduino.h>

class ARMIO {
public:
  ARMIO();
  ARMIO(const std::int8_t &arm_pulse, const std::int8_t &arm_feedback,
        const std::int8_t &wire_sig);
  ARMIO &operator=(const ARMIO &) = default;
  void init_pwm();
  void arm_set_position(const int &position, const bool &enable);
  void updatePD();

private:
  std::int8_t arm_pulse_pin;
  std::int8_t arm_feedback_pin;
  std::int8_t wire_sig_pin;

  unsigned long prev_msec;
  unsigned long wire_prev_msec;
  int servo_interval;

  // PID controller variables
  const float kp = 0.5;
  const float kd = 0.3;
  float previous_error;
  int target_position;

  // Read current arm position from feedback pin
  int getCurrentPosition();
};

#endif
