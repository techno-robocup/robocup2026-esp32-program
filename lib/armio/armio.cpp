#include "armio.hpp"
#include <algorithm>

ARMIO::ARMIO(const std::int8_t& arm_pulse, const std::int8_t& arm_feedback,
             const std::int8_t& wire_sig)
    : arm_pulse_pin(arm_pulse)
    , arm_feedback_pin(arm_feedback)
    , wire_sig_pin(wire_sig)
    // Note: LEDC channels 8 and 9 are intentionally reserved for ARMIO.
    // MOTORIO uses channels 0–3 via its own allocator; reserving 8 and 9 here
    // avoids conflicts and leaves 4–7 available for other peripherals.
    , arm_pulse_channel(8)
    , wire_sig_channel(9)
    , previous_error(0.0)
    , integral_sum(0.0)
    , target_position(2048) {}  // Start at middle position

ARMIO::ARMIO()
    : arm_pulse_pin(-1)
    , arm_feedback_pin(-1)
    , wire_sig_pin(-1)
    , arm_pulse_channel(-1)
    , wire_sig_channel(-1)
    , target_position(2048)
    , previous_error(0.0)
    , integral_sum(0.0) {}

void ARMIO::init_pwm() {
  // Configure LEDC for arm servo pulse (50Hz, 16-bit resolution)
  ledcSetup(arm_pulse_channel, 50, 16);
  ledcAttachPin(arm_pulse_pin, arm_pulse_channel);

  // Configure LEDC for wire signal (50Hz, 16-bit resolution)
  ledcSetup(wire_sig_channel, 50, 16);
  ledcAttachPin(wire_sig_pin, wire_sig_channel);

  pinMode(arm_feedback_pin, INPUT);
}

int ARMIO::getCurrentPosition() {
  // Read analog feedback from arm position sensor
  return analogRead(arm_feedback_pin);
}

void ARMIO::arm_set_position(const int& position, const bool& enable) {
  // Clamp position to valid range
  int clamped_position = position;
  if (clamped_position < 0) clamped_position = 0;        // Down
  if (clamped_position > 4095) clamped_position = 4095;  // Up

  target_position = clamped_position;

  // Convert pulse width in microseconds to PWM duty cycle
  // 50Hz = 20ms (20000us) period
  // Duty cycle = (pulse_width / period) * 65535
  int pwm_value;
  if (enable)
    pwm_value = 2400;  // tight
  else
    pwm_value = 500;  // slack

  uint32_t duty = (static_cast<uint32_t>(pwm_value) * 65535) / 20000;
  ledcWrite(wire_sig_channel, duty);
}

void ARMIO::updatePID() {
  int current_position = getCurrentPosition();
  float error = target_position - current_position;

  // Deadband: stop if close enough (±20 counts)
  if (error > -20 && error < 20) {
    // Stop motor: 1500µs pulse
    uint32_t duty = (1500 * 65535) / 20000;  // ~4915
    ledcWrite(arm_pulse_channel, duty);
    return;
  }

  // Proportional term
  float proportional = kp * error;

  // Integral term (accumulate error over time)
  integral_sum += error;
  float integral = ki * integral_sum;

  integral = min(1000.0f, max(-1000.0f, integral));  // Clamp integral term to prevent windup

  // Derivative term
  float derivative = error - previous_error;
  float derivative_term = kd * derivative;

  // Calculate PID output (motor speed correction)
  float pid_output = proportional + integral + derivative_term;

  // Convert PID to PWM: 1500µs (stop) ± pid_output
  int pulse_width = 1500 + (int)pid_output;

  // Clamp to valid servo range (1000-2000µs)
  if (pulse_width < 1000) pulse_width = 1000;
  if (pulse_width > 2000) pulse_width = 2000;

  // Convert pulse width to duty cycle
  uint32_t duty = (static_cast<uint32_t>(pulse_width) * 65535) / 20000;
  ledcWrite(arm_pulse_channel, duty);

  // Update for next iteration
  previous_error = error;
}
