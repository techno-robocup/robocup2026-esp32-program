#include <motorio.hpp>

MOTORIO::MOTORIO(const std::int8_t& _PIN, const int& _interval)
    : PIN(_PIN), interval(_interval), prev_msec(micros()) {
  pinMode(PIN, OUTPUT);
}

MOTORIO::MOTORIO() {}

void MOTORIO::run_msec(const int& msec) {
  unsigned long current_micros = micros();
  // Handle micros() overflow (wraps around every ~70 minutes)
  unsigned long elapsed = (current_micros >= prev_msec)
                              ? (current_micros - prev_msec)
                              : (0xFFFFFFFF - prev_msec + current_micros + 1);

  if (elapsed < (unsigned long)interval) {
    return;
  }
  digitalWrite(PIN, HIGH);
  delayMicroseconds(msec);
  digitalWrite(PIN, LOW);
  prev_msec = current_micros;
}
