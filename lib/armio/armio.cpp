#include "armio.hpp"

ARMIO::ARMIO(const std::int8_t& arm_pulse, const std::int8_t& arm_feedback,
             const std::int8_t& wire_sig)
    : arm_pulse_pin(arm_pulse)
    , arm_feedback_pin(arm_feedback)
    , wire_sig_pin(wire_sig)
    , prev_msec(micros())
    , wire_prev_msec(micros())
    , servo_interval(20000)
    ,  // 20ms interval for servo PWM
    kp(0.5)
    , ki(0)
    , kd(0.3)
    , previous_error(0.0)
    , integral(0.0)
    , target_position(2048) {}  // Start at middle position

ARMIO::ARMIO() {}

void ARMIO::init_pwm() {
  pinMode(arm_pulse_pin, OUTPUT);
  pinMode(arm_feedback_pin, INPUT);
  pinMode(wire_sig_pin, OUTPUT);
  digitalWrite(wire_sig_pin, LOW);
}

int ARMIO::getCurrentPosition() {
  // Read analog feedback from arm position sensor
  return analogRead(arm_feedback_pin);
}

void ARMIO::arm_set_position(const int& position) {
  // Clamp position to valid range
  int clamped_position = position;
  if (clamped_position < 0) clamped_position = 0;
  if (clamped_position > 4095) clamped_position = 4095;

  target_position = clamped_position;
}

void ARMIO::wire_tension_function(const bool& enable) {
  unsigned long current_micros = micros();
  unsigned long elapsed = (current_micros >= wire_prev_msec)
                              ? (current_micros - wire_prev_msec)
                              : (0xFFFFFFFF - wire_prev_msec + current_micros + 1);

  if (elapsed < (unsigned long)servo_interval) {
    return;
  }

  int pwm_value;
  if (enable)
    pwm_value = 2400;
  else
    pwm_value = 500;
  digitalWrite(wire_sig_pin, HIGH);
  delayMicroseconds(pwm_value);
  digitalWrite(wire_sig_pin, LOW);
  wire_prev_msec = current_micros;
}
void ARMIO::updatePID() {
  unsigned long current_micros = micros();
  // Handle micros() overflow (wraps around every ~70 minutes)
  unsigned long elapsed = (current_micros >= prev_msec)
                              ? (current_micros - prev_msec)
                              : (0xFFFFFFFF - prev_msec + current_micros + 1);

  if (elapsed < (unsigned long)servo_interval) {
    return;
  }

  int current_position = getCurrentPosition();
  float error = target_position - current_position;

  // Deadband: stop if close enough (±20 counts)
  if (error > -20 && error < 20) {
    digitalWrite(arm_pulse_pin, HIGH);
    delayMicroseconds(1500);  // Stop motor
    digitalWrite(arm_pulse_pin, LOW);
    prev_msec = current_micros;
    return;
  }

  // Proportional term
  float proportional = kp * error;

  // Integral term
  integral += error;
  if (integral > 1000 || integral < -1000) {
    integral = 0;
  }
  float integral_term = ki * integral;

  // Derivative term
  float derivative = error - previous_error;
  float derivative_term = kd * derivative;

  // Calculate PID output (motor speed correction)
  float pid_output = proportional + integral_term + derivative_term;

  // Convert PID to PWM: 1500µs (stop) ± pid_output
  int pulse_width = 1500 + (int)pid_output;

  // Clamp to valid servo range (1000-2000µs)
  if (pulse_width < 1000) pulse_width = 1000;
  if (pulse_width > 2000) pulse_width = 2000;

  digitalWrite(arm_pulse_pin, HIGH);
  delayMicroseconds(pulse_width);
  digitalWrite(arm_pulse_pin, LOW);

  // Update for next iteration
  previous_error = error;
  prev_msec = current_micros;
}
