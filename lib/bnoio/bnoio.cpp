#include "bnoio.hpp"
#include <Arduino.h>  // Includes FreeRTOS for vTaskDelay

BNOIO::BNOIO()
    : bno(Adafruit_BNO055(55, 0x28))
    , sda_pin_(21)
    , scl_pin_(22)
    , initialized_(false)
    , heading_(0)
    , roll_(0)
    , pitch_(0)
    , accel_x_(0)
    , accel_y_(0)
    , accel_z_(0) {}

BNOIO::BNOIO(int sda_pin, int scl_pin)
    : bno(Adafruit_BNO055(55, 0x28))
    , sda_pin_(sda_pin)
    , scl_pin_(scl_pin)
    , initialized_(false)
    , heading_(0)
    , roll_(0)
    , pitch_(0)
    , accel_x_(0)
    , accel_y_(0)
    , accel_z_(0) {}

bool BNOIO::init() {
  // Initialize I2C bus with specified pins (use standard 400kHz for BNO055)
  Wire.begin(sda_pin_, scl_pin_, 400000);  // 400kHz I2C clock (standard for sensors)

  Serial.println("[BNO] Attempting bno.begin()...");
  if (!bno.begin()) {
    Serial.println("[BNO] begin() failed!");
    initialized_ = false;
    return false;
  }
  Serial.println("[BNO] begin() successful!");
  
  Serial.println("[BNO] Setting external crystal...");
  bno.setExtCrystalUse(true);

  // Set to IMU mode (accelerometer + gyroscope fusion, no magnetometer)
  Serial.println("[BNO] Setting operation mode...");
  bno.setMode(OPERATION_MODE_IMUPLUS);
  
  Serial.println("[BNO] Initialization complete!");
  initialized_ = true;
  return true;
}

bool BNOIO::readSensor() {
  if (!initialized_) {
    return false;
  }

  sensors_event_t orientationData, accelerometerData;
  
  // Try to read sensor with timeout recovery
  static unsigned long last_successful_read = 0;
  unsigned long now = millis();
  
  // If no successful read for 5 seconds, reinitialize
  if (now - last_successful_read > 5000) {
    Serial.println("[BNO] No successful reads for 5s, reinitializing...");
    initialized_ = false;
    // Don't recursively call init, just reset flag
    // The next init() call will reinitialize
    return false;
  }
  
  // Try to get data - the library's getEvent doesn't return error codes
  // but we can at least avoid reading too fast
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  heading_ = orientationData.orientation.x;
  roll_ = orientationData.orientation.y;
  pitch_ = orientationData.orientation.z;

  accel_x_ = accelerometerData.acceleration.x;
  accel_y_ = accelerometerData.acceleration.y;
  accel_z_ = accelerometerData.acceleration.z;

  last_successful_read = now;
  return true;
}

float BNOIO::getHeading() const { return heading_; }

float BNOIO::getRoll() const { return roll_; }

float BNOIO::getPitch() const { return pitch_; }

float BNOIO::getAccelX() const { return accel_x_; }

float BNOIO::getAccelY() const { return accel_y_; }

float BNOIO::getAccelZ() const { return accel_z_; }

bool BNOIO::isInitialized() const { return initialized_; }
