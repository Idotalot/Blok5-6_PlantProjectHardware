#pragma once

#include <Arduino.h>

class PhSensor {
public:
  PhSensor(uint8_t analogPin, float adcRefVolt, int adcResolution, int samples)
    : pin(analogPin), refVolt(adcRefVolt), resolution(adcResolution), numSamples(samples) {}

  void setCalibration(float ph1, float v1, float ph2, float v2) {
    calPh1 = ph1; calV1 = v1; calPh2 = ph2; calV2 = v2;
  }

  float readVoltage() const {
    long sum = 0;
    for (int i = 0; i < numSamples; i++) {
      sum += analogRead(pin);
      delay(10);
    }
    float raw = (float)sum / numSamples;
    return (raw / (float)resolution) * refVolt;
  }

  float readPh() const {
    float v = readVoltage();
    float slope = (calPh2 - calPh1) / (calV2 - calV1);
    float offset = calPh1 - slope * calV1;
    return slope * v + offset;
  }

private:
  uint8_t pin;
  float refVolt;
  int resolution;
  int numSamples;
  float calPh1 = 7.0f;
  float calV1  = 1.882f;
  float calPh2 = 4.0f;
  float calV2  = 1.982f;
};


