#include <motorio.hpp>

MOTORIO::MOTORIO(const std::int8_t& _PIN, const int& _interval)
    : PIN(_PIN), interval(_interval), prev_msec(micros()) {
  pinMode(PIN, OUTPUT);
}

MOTORIO::MOTORIO() {}

// Note: Despite the function name, 'msec' is actually in microseconds (µs) for PWM pulse width
void MOTORIO::run_msec(const int& msec) {
  unsigned long current_micros = micros();
  // Handle micros() overflow (wraps around every ~70 minutes)
  unsigned long elapsed = (current_micros >= prev_msec)
                              ? (current_micros - prev_msec)
                              : (0xFFFFFFFF - prev_msec + current_micros + 1);

  if (elapsed < (unsigned long)interval) {
    return;
  }

  // Clamp pulse width to safe servo/ESC range (1000-2000µs)
  int pulse_width = msec;
  if (pulse_width < MOTOR_PWM_MIN_US) pulse_width = MOTOR_PWM_MIN_US;
  if (pulse_width > MOTOR_PWM_MAX_US) pulse_width = MOTOR_PWM_MAX_US;

  digitalWrite(PIN, HIGH);
  delayMicroseconds(pulse_width);
  digitalWrite(PIN, LOW);
  prev_msec = current_micros;
}
