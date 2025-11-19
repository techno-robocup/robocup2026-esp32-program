#include "usonicio.hpp"

UltrasonicIO::UltrasonicIO(int _trig, int _echo) : trig(_trig), echo(_echo) {
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
}

void UltrasonicIO::readUsonic(long* values) {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  // Add timeout to prevent blocking (max 30ms = ~5 meters)
  long duration = pulseIn(echo, HIGH, 30000);
  if (duration == 0) {
    duration = 30000;  // Set to max on timeout
  }
  // Convert to centimeters: (duration * 0.034) / 2
  // Simplified: (duration * 17) / 1000
  duration = (duration * 17) / 1000;
  *values = duration;
  return;
}
