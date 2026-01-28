#include <motorio.hpp>

int MOTORIO::channel_counter = 0;

MOTORIO::MOTORIO(const std::int8_t& _PIN, const int& _interval) : PIN(_PIN), interval(_interval) {
  // Prevent exceeding available LEDC channels (0-7 for MOTORIO, 8-15 reserved)
  if (channel_counter >= 8) {
    // Fatal error: too many MOTORIO instances
    Serial.println("ERROR: Cannot create more than 8 MOTORIO instances");
    ledc_channel = -1;
    return;
  }

  // Assign LEDC channel sequentially
  ledc_channel = channel_counter;
  channel_counter++;

  // Configure LEDC PWM
  // Frequency: 50Hz (20ms period for servo/motor control)
  // Resolution: 16-bit (0-65535)
  ledcSetup(ledc_channel, 50, 16);
  ledcAttachPin(PIN, ledc_channel);
}

MOTORIO::MOTORIO() : PIN(-1), ledc_channel(-1), interval(0) {}

void MOTORIO::run_msec(const int& msec) {
  if (ledc_channel < 0 || PIN < 0) {
    return;
  }
  // Convert pulse width in microseconds to PWM duty cycle
  // 50Hz = 20ms (20000us) period
  // Duty cycle = (pulse_width / period) * 65535
  // For 1500us pulse: (1500 / 20000) * 65535 ≈ 4915
  uint32_t duty = (static_cast<uint32_t>(msec) * 65535) / 20000;
  ledcWrite(ledc_channel, duty);
}
