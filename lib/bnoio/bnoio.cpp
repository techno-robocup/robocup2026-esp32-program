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
  // Initialize I2C bus with specified pins
  // Note: Wire.begin() can be called multiple times safely in ESP32 Arduino
  Wire.begin(sda_pin_, scl_pin_);

  if (!bno.begin()) {
    initialized_ = false;
    return false;
  }

  // BNO055 requires time to initialize after begin()
  vTaskDelay(pdMS_TO_TICKS(1000));
  bno.setExtCrystalUse(true);

  // Set to IMU mode (accelerometer + gyroscope fusion, no magnetometer)
  bno.setMode(Adafruit_BNO055::OPERATION_MODE_IMUPLUS);

  initialized_ = true;
  return true;
}

bool BNOIO::readSensor() {
  if (!initialized_) {
    return false;
  }

  sensors_event_t orientationData, accelerometerData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  heading_ = orientationData.orientation.x;
  roll_ = orientationData.orientation.y;
  pitch_ = orientationData.orientation.z;

  accel_x_ = accelerometerData.acceleration.x;
  accel_y_ = accelerometerData.acceleration.y;
  accel_z_ = accelerometerData.acceleration.z;

  return true;
}

float BNOIO::getHeading() const { return heading_; }

float BNOIO::getRoll() const { return roll_; }

float BNOIO::getPitch() const { return pitch_; }

float BNOIO::getAccelX() const { return accel_x_; }

float BNOIO::getAccelY() const { return accel_y_; }

float BNOIO::getAccelZ() const { return accel_z_; }

bool BNOIO::isInitialized() const { return initialized_; }
